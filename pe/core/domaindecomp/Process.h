//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/Process.h
 *  \brief Header file for the Process class
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

#ifndef _PE_CORE_DOMAINDECOMP_PROCESS_H_
#define _PE_CORE_DOMAINDECOMP_PROCESS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <memory>
#include <pe/core/Configuration.h>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/MPI.h>
#include <pe/core/RecvBuffer.h>
#include <pe/core/SendBuffer.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Byte.h>
#include <pe/util/NonCopyable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Remote MPI process.
 * \ingroup domaindecomp
 *
 * \section remote_process_general General
 *
 * The Process class is the basic component for the MPI parallelization of the physics engine.
 * It represents a remote MPI process connected to the local process via MPI. The two processes
 * are separated by an arbitrarily shaped boundary. Most often, this boundary is shaped like a
 * plane that devides the global space in two half spaces. One of these half spaces is the half
 * space of the local process, the other half space is the half space of the remote MPI process.
 * The plane is represented by the following equation:
 *
 *                                \f[ ax + by + cz = d , \f]
 *
 * where \a a, \a b and \a c are the x, y and z component of the normal vector. The normal
 * \a n of the plane is a normalized vector that always points towards the half space of
 * the remote process. \a d is the distance/displacement from the origin of the global world
 * frame to the plane. A positive value of \a d indicates that the origin of the global world
 * frame is inside the local process, whereas a negative value of \a d indicates that the
 * origin is inside the remote process. A value of 0 indicates that the origin is on the
 * surface of the plane. Any point on the surface of the plane is considered part of the
 * process with the lower rank.
 *
 * \image html process.png
 * \image latex process.eps "Remote MPI process" width=520pt
 *
 * The part of the simulation world represented by this remote process can not be directly
 * accessed by the local process since the data may be stored in a completely different
 * memory on a different machine. Therefore the only way to exchange rigid bodies close to
 * the process boundary is a communication via the message passing interface (MPI). This
 * communication is automatically initiated during a time step of the simulation or explicitly
 * by the World::synchronize() function that simultaneously starts the data exchange with all
 * adjacent processes in order to synchronize the connected processes.
 */
class Process : public NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef SendBuffer<NoEndiannessConversion>  SendBuff;  //!< Type of the MPI send buffer.
   typedef RecvBuffer<NoEndiannessConversion>  RecvBuff;  //!< Type of the MPI receive buffer.
   //**********************************************************************************************

private:
   //**Type definitions****************************************************************************
   typedef std::auto_ptr<ProcessGeometry>      Geometry;  //!< Handle for the process geometry.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Process( int rank, Geometry geometry, const Vec3& offset );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Process();
   //@}
   //**********************************************************************************************

public:
   //**MPI Send/Receive functions******************************************************************
   /*!\name MPI Send/Receive functions */
   //@{
   void send   ( int tag, MPI_Request* request );
   void receive( int tag );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline int             getRank  ()     const;
   inline const Vec3&     getOffset()     const;
   inline const SendBuff& getSendBuffer() const;
   inline SendBuff&       getSendBuffer();
   inline const RecvBuff& getRecvBuffer() const;
   inline RecvBuff&       getRecvBuffer();
   inline const ProcessGeometry* getGeometry() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool containsPoint        ( const Vec3& gpos ) const;
   inline bool containsPointStrictly( const Vec3& gpos ) const;

   inline bool intersectsWith( ConstBodyID     b ) const;
   inline bool intersectsWith( ConstSphereID   s ) const;
   inline bool intersectsWith( ConstBoxID      b ) const;
   inline bool intersectsWith( ConstCapsuleID  c ) const;
   inline bool intersectsWith( ConstCylinderID c ) const;
   inline bool intersectsWith( ConstUnionID    u ) const;

          bool requires( ConstBodyID       b ) const;
          bool requires( ConstSphereID     s ) const;
          bool requires( ConstBoxID        b ) const;
          bool requires( ConstCapsuleID    c ) const;
          bool requires( ConstCylinderID   c ) const;
          bool requires( ConstUnionID      u ) const;

          bool requires( ConstAttachableID a ) const;
          bool requires( ConstGravityID    g ) const;
          bool requires( ConstSpringID     s ) const;

   inline void clear();
	inline bool hasData() const;
	//@}
	//**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Geometry geometry_;  //!< The geometry of the remote process.
                        /*!< The geometry of the remote process, i.e. the physical expansion of
                             the process, is used to evaluate whether rigid bodies are (partially)
                             contained in the remote process or whether global coordinates fall
                             into the space occupied by the remote process. The geometry/expansion
                             is assigned during construction of the process according to the
                             strategy pattern and managed by this handle. */
   int      rank_;      //!< Rank of the remote MPI process.
   Vec3     offset_;    //!< Offset between the local and this remote process.
   SendBuff send_;      //!< Send buffer to the remote MPI process.
   RecvBuff recv_;      //!< Receive buffer from the remote MPI process.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend void connect_backend( int rank, std::auto_ptr<ProcessGeometry> geometry, const Vec3& offset );
   friend void disconnect     ( int rank );
   friend class MPICommunication;
   friend class ProcessManager;
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
/*!\brief Returns the rank of the remote process.
 *
 * \return The rank of the remote process.
 */
inline int Process::getRank() const
{
   return rank_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the offset between the local and the remote process.
 *
 * \return The offset between the local and the remote process.
 */
inline const Vec3& Process::getOffset() const
{
   return offset_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a read-only reference to the MPI receive buffer.
 *
 * \return A read-only reference to the MPI receive buffer.
 */
inline const Process::RecvBuff& Process::getRecvBuffer() const
{
   return recv_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a read-write reference to the MPI receive buffer.
 *
 * \return A read-write reference to the MPI receive buffer.
 */
inline Process::RecvBuff& Process::getRecvBuffer()
{
   return recv_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a read-only reference to the MPI send buffer.
 *
 * \return A read-only reference to the MPI send buffer.
 */
inline const Process::SendBuff& Process::getSendBuffer() const
{
   return send_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a read-write reference to the MPI send buffer.
 *
 * \return A read-write reference to the MPI send buffer.
 */
inline Process::SendBuff& Process::getSendBuffer()
{
   return send_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a read-only pointer to the process geometry.
 *
 * \return A read-only pointer to the process geometry.
 */
inline const ProcessGeometry* Process::getGeometry() const
{
   return geometry_.get();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained in the remote process.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the remote process, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the remote process. If
 * the point is located on the surface of the remote process it is considered to be part of the
 * process domain.
 */
inline bool Process::containsPoint( const Vec3& gpos ) const
{
   return geometry_->containsPoint( gpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained strictly in the remote process.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the remote process, \a false if not.
 *
 * This function tests whether the given global coordinate is contained strictly in the remote
 * process. If the point is located on the surface of the remote process it is considered to be
 * \em not part of the process domain.
 */
inline bool Process::containsPointStrictly( const Vec3& gpos ) const
{
   return geometry_->containsPointStrictly( gpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given rigid body intersects with the remote process.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the remote process, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the remote
 * process. In case the body is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false. Note that it is not possible to test infinite rigid bodies (as for instance
 * planes). The attempt to test an infinite rigid body results in a \a std::invalid_argument
 * exception.
 */
inline bool Process::intersectsWith( ConstBodyID b ) const
{
   return geometry_->intersectsWith( b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sphere intersects with the remote process.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the remote process, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the remote
 * process. In case the sphere is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Process::intersectsWith( ConstSphereID s ) const
{
   return geometry_->intersectsWith( s );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given box intersects with the remote process.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the remote process, \a false if not.
 *
 * This function tests whether the given box is partially contained in the remote
 * process. In case the box is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Process::intersectsWith( ConstBoxID b ) const
{
   return geometry_->intersectsWith( b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given capsule intersects with the remote process.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the remote process, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the remote
 * process. In case the capsule is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Process::intersectsWith( ConstCapsuleID c ) const
{
   return geometry_->intersectsWith( c );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given cylinder intersects with the remote process.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the remote process, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the remote
 * process. In case the cylinder is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Process::intersectsWith( ConstCylinderID c ) const
{
   return geometry_->intersectsWith( c );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given union intersects with the remote process.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the remote process, \a false if not.
 *
 * This function tests whether the given union is partially contained in the remote
 * process. In case the union is partially contained in the remote process (i.e. overlaps
 * the boundary to the remote process) the function returns \a true, otherwise it returns
 * \a false.
 */
inline bool Process::intersectsWith( ConstUnionID u ) const
{
   return geometry_->intersectsWith( u );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the send buffer of the communication channel to the remote MPI process.
 *
 * \return void
 */
inline void Process::clear()
{
   send_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the channel contains received data from the remote process.
 *
 * \return \a true if the channel still contains data from the remote process, \a false if not.
 *
 * This function returns whether the communication channel still has received data from the
 * connected remote MPI process. In case the channel still contains data, the function returns
 * \a true, else it returns \a false.
 */
inline bool Process::hasData() const
{
   return !recv_.isEmpty();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Rigid body operators */
//@{
std::ostream& operator<<( std::ostream& os, const Process& p );
std::ostream& operator<<( std::ostream& os, ConstProcessID p );
//@}
//*************************************************************************************************

} // namespace pe

#endif
