//=================================================================================================
/*!
 *  \file src/core/io/BodySimpleAsciiWriter.cpp
 *  \brief Writer for simple rigid body Ascii files
 *
 *  Copyright (C) 2011-2012 Tobias Preclik
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

#include <algorithm>
#include <boost/numeric/conversion/cast.hpp>
#include <pe/core/io/BodySimpleAsciiWriter.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/domaindecomp/Process.h>
#include <vector>


namespace pe {

//=================================================================================================
//
//  I/O FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dumps a textual description of all rigid bodies in the simulation world.
 *
 * \param filename The filename to write to.
 * \return void
 *
 * Dumps a textual description of all rigid bodies in the simulation world by communicating
 * all body descriptions to the root process. The bodies are sorted by user ID.
 */
void BodySimpleAsciiWriter::writeFile( const char* filename ) {
   const int root( MPISettings::root() );
   const int myRank( MPISettings::rank() );

   typedef CollisionSystem<Config>::BS BS;
   const BS& bodystorage( theCollisionSystem()->getBodyStorage() );

   if( myRank == root ) {
      Process::RecvBuff buffer;

      // Buffer local bodies
      Process::SendBuff bufferLocal;
      for( BS::ConstIterator body = bodystorage.begin(); body != bodystorage.end(); ++body ) {
         if( !body->isRemote() ) {
            marshalDynamically( bufferLocal, *(*body) );
         }
      }

#if HAVE_MPI
      const int          size( MPISettings::size() );
      const MPI_Comm     comm( MPISettings::comm() );
      const MPI_Datatype type( MPITrait<byte>::getType() );

      MPI_Status status;
      int count( 0 ), totalcount( boost::numeric_cast<int>( bufferLocal.size() ) );

      for( int i = 0; i < size; ++i ) {
         if( i == root )
            continue;

         MPI_Probe( i, mpitagBodySimpleAsciiWriter, comm, &status );
         MPI_Get_count( &status, type, &count );
         totalcount += count;
      }

      buffer.resize( totalcount );
      byte* dst = buffer.ptr();
      memcpy(dst, bufferLocal.ptr(), bufferLocal.size() );
      dst += bufferLocal.size();

      for( int i = 0; i < size; ++i ) {
         if( i == root )
            continue;

         MPI_Probe( i, mpitagBodySimpleAsciiWriter, comm, &status );
         MPI_Get_count( &status, type, &count );
         MPI_Recv( dst, count, type, i, mpitagBodySimpleAsciiWriter, comm, &status );

         dst += count;
      }
#else
      buffer.resize( bufferLocal.size() );
      memcpy(buffer.ptr(), bufferLocal.ptr(), bufferLocal.size() );
#endif

      std::vector<Sphere::Parameters>   spheres;
      std::vector<Box::Parameters>      boxes;
      std::vector<Capsule::Parameters>  capsules;
      std::vector<Cylinder::Parameters> cylinders;
      std::vector<Plane::Parameters>    planes;
      std::vector<Union::Parameters>    unions;

      while( !buffer.isEmpty() ) {
         GeomType geomType;
         unmarshal( buffer, geomType );

         switch( geomType ) {
            case sphereType: {
               Sphere::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               spheres.push_back( subobjparam );
               break;
            }
            case boxType: {
               Box::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               boxes.push_back( subobjparam );
               break;
            }
            case capsuleType: {
               Capsule::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               capsules.push_back( subobjparam );
               break;
            }
            case cylinderType: {
               Cylinder::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               cylinders.push_back( subobjparam );
               break;
            }
            case planeType: {
               Plane::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               planes.push_back( subobjparam );
               break;
            }
            case unionType: {
               Union::Parameters subobjparam;
               unmarshal( buffer, subobjparam, false );
               unions.push_back( subobjparam );
               break;
            }
            default: {
               throw std::runtime_error( "Unknown geometry type" );
            }
         }
      }

      std::sort( spheres.begin(), spheres.end(), RigidBodyCompare() );
      std::sort( boxes.begin(), boxes.end(), RigidBodyCompare() );
      std::sort( capsules.begin(), capsules.end(), RigidBodyCompare() );
      std::sort( cylinders.begin(), cylinders.end(), RigidBodyCompare() );
      std::sort( planes.begin(), planes.end(), RigidBodyCompare() );
      std::sort( unions.begin(), unions.end(), RigidBodyCompare() );

      std::ofstream fout( filename );

      fout << "Spheres (" << spheres.size() << "):\n";
      for( std::vector<Sphere::Parameters>::iterator it = spheres.begin(); it != spheres.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
         fout << "r    = " << it->radius_ << "\n";
      }

      fout << "\nUnions (" << unions.size() << "):\n";
      for( std::vector<Union::Parameters>::iterator it = unions.begin(); it != unions.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
      }

      fout << "\nBoxes (" << boxes.size() << "):\n";
      for( std::vector<Box::Parameters>::iterator it = boxes.begin(); it != boxes.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
         fout << "len  = " << it->lengths_ << "\n";
      }

      fout << "\nCapsules (" << capsules.size() << "):\n";
      for( std::vector<Capsule::Parameters>::iterator it = capsules.begin(); it != capsules.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
         fout << "r    = " << it->radius_ << "\n";
         fout << "len  = " << it->length_ << "\n";
      }

      fout << "\nCylinders (" << cylinders.size() << "):\n";
      for( std::vector<Cylinder::Parameters>::iterator it = cylinders.begin(); it != cylinders.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
         fout << "r    = " << it->radius_ << "\n";
         fout << "len  = " << it->length_ << "\n";
      }

      fout << "\nPlanes (" << planes.size() << "):\n";
      for( std::vector<Plane::Parameters>::iterator it = planes.begin(); it != planes.end(); ++it ) {
         //fout << "sid  = " << it->sid_ << "\n";
         fout << "uid  = " << it->uid_ << "\n";
         fout << "gpos = " << it->gpos_ << "\n";
         fout << "q    = " << it->q_ << "\n";
         fout << "v    = " << it->v_ << "\n";
         fout << "w    = " << it->w_ << "\n";
         // reconstruct n and dist:
         fout << "n    = " << (it->q_).rotate( Vec3( 0, 0, 1 ) ) << "\n";
         fout << "dist = " << trans( (it->q_).rotate( Vec3( 0, 0, 1 ) ) ) * it->gpos_ << "\n";
      }
   }
   else {
#if HAVE_MPI
      Process::SendBuff buffer;

      // Send local bodies to root
      for( BS::ConstIterator body = bodystorage.begin(); body != bodystorage.end(); ++body ) {
         if( !body->isRemote() && !body->isGlobal() ) {
            marshalDynamically( buffer, *(*body) );
         }
      }

      const int          size( boost::numeric_cast<int>( buffer.size() ) );
      const MPI_Comm     comm( MPISettings::comm() );

      MPI_Ssend( buffer.ptr(), size, MPITrait<byte>::getType(), root, mpitagBodySimpleAsciiWriter, comm );
#endif
   }
}
//*************************************************************************************************

} // namespace pe
