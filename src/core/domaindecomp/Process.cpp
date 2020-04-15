//=================================================================================================
/*!
 *  \file src/core/domaindecomp/Process.cpp
 *  \brief Source file for the Process class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <ostream>
#include <stdexcept>
#include <string>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPITrait.h>
#include <pe/core/ParallelTrait.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Null.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Process class.
 *
 * \param rank The rank of the remote process.
 * \param geometry The geometry/physical expansion of the remote MPI process.
 * \param offset The offset between the two processes.
 */
Process::Process( int rank, Geometry geometry, const Vec3& offset )
   : geometry_( geometry )  // The geometry of the remote process
   , rank_  ( rank   )      // Rank of the remote MPI process
   , offset_( offset )      // Offset between the two processes.
   , send_  ( 10000  )      // Send buffer to the remote MPI process
   , recv_  ()              // Receive buffer from the remote MPI process
{
   // Checking the process geometry
   // Since the process constructor is never directly called but only used in a small number
   // of functions that already check the process arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( geometry_.get() != NULL, "Invalid process geometry detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Process class.
 */
Process::~Process()
{}
//*************************************************************************************************




//=================================================================================================
//
//  MPI SEND/RECEIVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Non-blocking send operation to the connected MPI process.
 *
 * \param tag Message tag for the MPI communication.
 * \param request MPI request handle for the non-blocking send operation.
 * \return void
 *
 * This function sends the contents of the send buffer across the MPI channel to the connected
 * MPI process.
 */
void Process::send( int tag, MPI_Request* request )
{
   const int          size( static_cast<int>( send_.size() ) );
   const MPI_Comm     comm( MPISettings::comm() );
   const MPI_Datatype type( MPITrait<byte>::getType() );

   MPI_Isend( send_.ptr(),  // Initial address of the send buffer
              size,         // Number of elements in the send buffer
              type,         // Data of each element in the send buffer
              rank_,        // Rank of the destination process
              tag,          // MPI communication tag
              comm,         // The MPI communicator
              request );    // The communication request
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Blocking receive operation from the connected MPI process.
 *
 * \param tag Message tag for the MPI communication.
 * \return void
 *
 * This function receives data from the connected MPI process.
 */
void Process::receive( int tag )
{
   const MPI_Comm     comm( MPISettings::comm() );
   const MPI_Datatype type( MPITrait<byte>::getType() );
   MPI_Status status;
   int count( 0 );

   // Estimating the size of the MPI message
   MPI_Probe( rank_, tag, comm, &status );
   MPI_Get_count( &status,   // The communication status
                  type,      // Data type of the elements to be received
                  &count );  // The number of elements in the MPI message

   // Receiving the MPI message
   recv_.resize( count );
   MPI_Recv( recv_.ptr(),  // Initial address of the receive buffer
             count,        // Number of elements to be received
             type,         // Data type of each receive buffer element
             rank_,        // Rank of the source process
             tag,          // MPI communication tag
             comm,         // The MPI communicator
             &status );    // The communication status
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the given rigid body is required by the remote process.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body is required by the remote process, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is required by the remote process. In case
 * either the body itself or any directly attached body is (partially) contained in the remote
 * process (i.e. overlaps the boundary to the remote process) the function returns \a true,
 * otherwise it returns \a false. Note that it is not possible to test infinite rigid bodies
 * (as for instance planes). The attempt to test an infinite rigid body results in a
 * \a std::invalid_argument exception.
 */
bool Process::requires( ConstBodyID b ) const
{
   if( intersectsWith( b ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( b->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( b->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sphere is required by the remote process.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere is required by the remote process, \a false if not.
 *
 * This function tests whether the given sphere is required by the remote process. In case either
 * the sphere itself or any directly attached body is (partially) contained in the remote process
 * (i.e. overlaps the boundary to the remote process) the function returns \a true, otherwise it
 * returns \a false.
 */
bool Process::requires( ConstSphereID s ) const
{
   if( intersectsWith( s ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( s->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( s->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given box is required by the remote process.
 *
 * \param b The box to be tested.
 * \return \a true if the box is required by the remote process, \a false if not.
 *
 * This function tests whether the given box is required by the remote process. In case either
 * the box itself or any directly attached body is (partially) contained in the remote process
 * (i.e. overlaps the boundary to the remote process) the function returns \a true, otherwise
 * it returns \a false.
 */
bool Process::requires( ConstBoxID b ) const
{
   if( intersectsWith( b ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( b->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( b->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given capsule is required by the remote process.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule is required by the remote process, \a false if not.
 *
 * This function tests whether the given capsule is required by the remote process. In case either
 * the capsule itself or any directly attached body is (partially) contained in the remote process
 * (i.e. overlaps the boundary to the remote process) the function returns \a true, otherwise it
 * returns \a false.
 */
bool Process::requires( ConstCapsuleID c ) const
{
   if( intersectsWith( c ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( c->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( c->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given cylinder is required by the remote process.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder is required by the remote process, \a false if not.
 *
 * This function tests whether the given cylinder is required by the remote process. In case either
 * the cylinder itself or any directly attached body is (partially) contained in the remote process
 * (i.e. overlaps the boundary to the remote process) the function returns \a true, otherwise it
 * returns \a false.
 */
bool Process::requires( ConstCylinderID c ) const
{
   if( intersectsWith( c ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( c->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( c->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given union is required by the remote process.
 *
 * \param u The union to be tested.
 * \return \a true if the union is required by the remote process, \a false if not.
 *
 * This function tests whether the given union is required by the remote process. In case either
 * the union itself or any directly attached body is (partially) contained in the remote process
 * (i.e. overlaps the boundary to the remote process) the function returns \a true, otherwise it
 * returns \a false.
 */
bool Process::requires( ConstUnionID u ) const
{
   if( intersectsWith( u ) ) return true;

   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator begin( u->beginAttachedBodies() );
   const ConstAttachedBodyIterator end  ( u->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given attachable is required by the remote process.
 *
 * \param a The attachable to be tested.
 * \return \a true if the attachable is required by the remote process, \a false if not.
 *
 * This function tests whether the given attachable is required by the remote process. In case
 * one of the directly attached body is (partially) contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns \a false.
 */
bool Process::requires( ConstAttachableID a ) const
{
   typedef Attachable::ConstIterator  ConstIterator;
   const ConstIterator begin( a->begin() );
   const ConstIterator end  ( a->end()   );

   for( ConstIterator attachedBody=begin; attachedBody!=end; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) )
         return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given gravity force generator is required by the remote process.
 *
 * \param g The gravity force generator to be tested.
 * \return \a true if the gravity force generator is required by the remote process, \a false if not.
 *
 * This function tests whether the given gravity force generator is required by the remote process.
 * In case the directly attached body is (partially) contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns \a false.
 */
bool Process::requires( ConstGravityID g ) const
{
   return intersectsWith( g->getBody() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given spring is required by the remote process.
 *
 * \param s The spring to be tested.
 * \return \a true if the spring is required by the remote process, \a false if not.
 *
 * This function tests whether the given spring is required by the remote process. In case one of
 * the two directly attached bodies is (partially) contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns \a false.
 */
bool Process::requires( ConstSpringID s ) const
{
   return ( intersectsWith( s->getBody1() ) || intersectsWith( s->getBody2() ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of the process.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the process output.
 * \return void
 */
void Process::print( std::ostream& os, const char* tab ) const
{
   std::string longtab( tab );
   longtab.append( "   " );

   os << tab << "Process boundary between local process " << MPISettings::rank()
      << " and remote process " << rank_ << ":\n";
   geometry_->print( os, longtab.c_str() );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for remote MPI processes.
 * \ingroup mpi
 *
 * \param os Reference to the output stream.
 * \param p Reference to a constant process object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Process& p )
{
   os << "--" << pe_BROWN << "REMOTE PROCESS PARAMETERS" << pe_OLDCOLOR
      << "-----------------------------------------------------\n";
   p.print( os, " " );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for remote MPI processes.
 * \ingroup mpi
 *
 * \param os Reference to the output stream.
 * \param p Constant process handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstProcessID p )
{
os << "--" << pe_BROWN << "REMOTE PROCESS PARAMETERS" << pe_OLDCOLOR
      << "-----------------------------------------------------\n";
   p->print( os, " " );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
