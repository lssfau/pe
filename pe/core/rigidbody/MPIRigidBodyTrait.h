//=================================================================================================
/*!
 *  \file pe/core/MPIRigidBodyTrait.h
 *  \brief Base class for a specialization of the RigidBodyTrait class template for MPI parallel solvers.
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

#ifndef _PE_CORE_RIGIDBODY_MPIRIGIDBODYTRAIT_H_
#define _PE_CORE_RIGIDBODY_MPIRIGIDBODYTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/rigidbody/RigidBodyBase.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Twist.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>
#include <pe/util/Vector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for a specialization of the RigidBodyTrait class template for MPI parallel solvers.
 * \ingroup rigid_body
 *
 * This class adds functionality to track which process owns the rigid body and which processes
 * have remote copies of the body in case it is owned by this process.
 */
class MPIRigidBodyTrait : public RigidBodyBase
{
protected:
   //**Type definitions****************************************************************************
   typedef RigidBodyBase                Parent;          //!< The type of the parent class.
   typedef Process*                     ProcessID;       //!< Handle for a remote process.
   typedef const Process*               ConstProcessID;  //!< Handle for a constant remote process.
   typedef PtrVector<Process,NoDelete>  Processes;       //!< Vector for remote MPI processes the rigid body is contained in.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Processes::Iterator       ProcessIterator;       //!< Iterator over the connected processes.
   typedef Processes::ConstIterator  ConstProcessIterator;  //!< ConstIterator over the connected processes.
   typedef size_t                    SizeType;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit MPIRigidBodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~MPIRigidBodyTrait();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline int getOwnerRank() const;
   inline ProcessID getOwnerHandle() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setOwner( int rank, ProcessID handle );
   //@}
   //**********************************************************************************************

   //**Process functions***************************************************************************
   /*!\name Process functions */
   //@{
   inline void                 registerProcess  ( ProcessID process );
   inline void                 deregisterProcess( ProcessID process );
   inline bool                 isRegistered( ConstProcessID process ) const;
   inline bool                 hasProcesses  () const;
   inline ProcessIterator      beginProcesses();
   inline ConstProcessIterator beginProcesses() const;
   inline ProcessIterator      endProcesses  ();
   inline ConstProcessIterator endProcesses  () const;
   inline SizeType             sizeProcesses () const;
   inline void                 clearProcesses();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Processes processes_;    //!< Vector of all processes the rigid body intersects with.
   int       ownerRank_;    //!< Rank of the process owning the rigid body.
   ProcessID ownerHandle_;  //!< handle of the process owning the rigid body.
#ifndef NDEBUG
public:
   bool      debugFlag_;    //!< Flag for debugging purposes.
private:
#endif
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the MPIRigidBodyTrait constructor.
 *
 * \param body The body ID of this rigid body.
 */
inline MPIRigidBodyTrait::MPIRigidBodyTrait( BodyID body )
   : Parent( body )
   , ownerRank_( -1 )
   , ownerHandle_( 0 )
#ifndef NDEBUG
   , debugFlag_( false )
#endif
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a new remote process the rigid body is contained in.
 *
 * \param process The remote process to be registered with the rigid body.
 * \return void
 *
 * This function registers the given remote process with the rigid body. In case the process is
 * already registered, it is not registered again. This call has linear complexity.
 */
inline void MPIRigidBodyTrait::registerProcess( ProcessID process )
{
   if( !isRegistered( process ) )
      processes_.pushBack( process );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a remote process from the rigid body.
 *
 * \param process The remote process to be deregistered from the rigid body.
 * \return void
 *
 * This function deregisters the given remote process from the rigid body. This call has linear
 * complexity.
 */
inline void MPIRigidBodyTrait::deregisterProcess( ProcessID process )
{
   ProcessIterator pos = std::find( processes_.begin(), processes_.end(), process );
   if( pos != processes_.end() )
      processes_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks whether the given remote process is registered with the rigid body.
 *
 * \param process The remote process that possibly registered with the rigid body.
 * \return \a true if the given process is registered with the rigid body, \a false if not.
 *
 * This call has linear complexity.
 */
inline bool MPIRigidBodyTrait::isRegistered( ConstProcessID process ) const
{
   if( std::find( processes_.begin(), processes_.end(), process ) == processes_.end() )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether any processes are registered with the rigid body.
 *
 * \return \a true if at least one process is registered with the rigid body, \a false if not.
 */
inline bool MPIRigidBodyTrait::hasProcesses() const
{
   return !processes_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first remote process the rigid body is contained in.
 *
 * \return Iterator to the first remote process.
 */
inline MPIRigidBodyTrait::ProcessIterator MPIRigidBodyTrait::beginProcesses()
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first remote process the rigid body is contained in.
 *
 * \return Iterator to the first remote process.
 */
inline MPIRigidBodyTrait::ConstProcessIterator MPIRigidBodyTrait::beginProcesses() const
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last process the rigid body is contained in.
 *
 * \return Iterator just past the last remote process.
 */
inline MPIRigidBodyTrait::ProcessIterator MPIRigidBodyTrait::endProcesses()
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last process the rigid body is contained in.
 *
 * \return Iterator just past the last remote process.
 */
inline MPIRigidBodyTrait::ConstProcessIterator MPIRigidBodyTrait::endProcesses() const
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of registered processes.
 *
 * \return The number of registered processes.
 */
inline MPIRigidBodyTrait::SizeType MPIRigidBodyTrait::sizeProcesses() const
{
   return processes_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all registered remote processes from the rigid body.
 *
 * \return void
 *
 * This function clears all references to remote processes the rigid body may be contained in.
 */
inline void MPIRigidBodyTrait::clearProcesses()
{
   processes_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the rank of the owner process.
 *
 * \return The rank of the owner process.
 *
 * If not yet set by the collision system this function returns -1.
 */
inline int MPIRigidBodyTrait::getOwnerRank() const
{
   return ownerRank_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the handle of the owner process.
 *
 * \return The handle of the owner process.
 *
 * If not yet set by the collision system this function returns a NULL pointer.
 */
inline ProcessID MPIRigidBodyTrait::getOwnerHandle() const
{
   return ownerHandle_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the parent process.
 * \param rank The rank of the parent process.
 * \param process The handle of the parent process or NULL if not available.
 *
 * \return void
 *
 * If process is not NULL then its rank must match.
 */
inline void MPIRigidBodyTrait::setOwner( int rank, ProcessID handle )
{
   ownerRank_ = rank;
   ownerHandle_ = handle;

   pe_INTERNAL_ASSERT( handle == 0 || handle->getRank() == rank, "Ranks mismatch." );
}
//*************************************************************************************************

} // namespace pe

#endif
