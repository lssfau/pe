//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/MPICommunication.h
 *  \brief TODO
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_COLLISIONSYSTEM_MPICOMMUNICATION_H_
#define _PE_CORE_COLLISIONSYSTEM_MPICOMMUNICATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/Configuration.h>
#include <pe/core/MPI.h>
#include <pe/util/Vector.h>
#include <pe/util/TransferMeter.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
class PE_PROTECTED MPICommunication
{
public:
   //**Type definitions****************************************************************************
   typedef ProcessStorage<Config>      PS;               //!< Type of the process storage.
   typedef Config::ProcessType         ProcessType;      //!< Type of the remote processes.
   typedef Config::ProcessID           ProcessID;        //!< Handle for a remote process.
   typedef Config::ConstProcessID      ConstProcessID;   //!< Handle for a constant remote process.
   typedef PS::Iterator                Iterator;         //!< Iterator type of the process storage.
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit MPICommunication( PS& processstorage, Domain& domain );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   //~MPICommunication();
   //@}
   //**********************************************************************************************

protected:

   //**Communication functions*********************************************************************
   /*!\name Communication functions */
   //@{
   void communicate( int tag );
   ProcessID receiveNextMessage( int tag );
   //@}
   //**********************************************************************************************

   //**Send functions****************************************************************************
   /*!\name Send functions */
   //@{
   //@}
   //**********************************************************************************************

   //**Receive functions****************************************************************************
   /*!\name Receive functions */
   //@{
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   std::pair<int, ProcessID> findOwner( const Vec3& gpos );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   PS& processstorage_;         //!< Reference to the process storage.
   Domain& domain_;             //!< Reference to the local subdomain.
   Vector<MPI_Request> requests_;
   Vector<MPI_Status> stats_;
   TransferMeter sentOverall_, receivedOverall_;
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
