//=================================================================================================
/*!
 *  \file src/core/BodyBinaryReader.cpp
 *  \brief Reader for rigid body binary parameter files
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

#include <pe/core/BodyBinaryReader.h>
#include <pe/core/Marshalling.h>
#include <pe/core/MPISettings.h>
#include <pe/core/ProfilingSection.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/World.h>
#include <pe/util/logging/DebugSection.h>


namespace pe {

//=================================================================================================
//
//  I/O FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reads in a binary rigid body parameter file.
 * \param filename The filename of the parameter file to read in.
 * \return void
 */
void BodyBinaryReader::readFile( const char* filename ) {
   timing::WcTimer timeAll, timeOpen, timeReadAll, timeReadLocal;
   size_t sizeReadAll( 0 ), sizeReadLocal( 0 );
   pe_PROFILING_SECTION {
      timeAll.start();
   }

   std::string filenameCopy( filename );

   pe_PROFILING_SECTION {
      timeOpen.start();
   }

#if HAVE_MPI
   MPI_Status status;
   MPI_File fh;
   if ( MPI_File_open( MPISettings::comm(), &filenameCopy[0], MPI_MODE_RDONLY, MPI_INFO_NULL, &fh ) != MPI_SUCCESS )
      throw std::runtime_error( "Cannot open file." );
#else
   std::ifstream fh( &filenameCopy[0], std::ifstream::binary );
   if( !fh )
      throw std::runtime_error( "Cannot open file." );
#endif

   pe_PROFILING_SECTION {
      timeOpen.end();
   }

   // read until the position in the file where the number of processors used to write the file is stored
   header_.resize( 9*sizeof(byte) + 1*sizeof(uint32_t) );

   pe_PROFILING_SECTION {
      timeReadAll.start();
   }

#if HAVE_MPI
   MPI_File_read_all( fh, header_.ptr(), static_cast<int>( header_.size() ), MPI_BYTE, &status );
#else
   fh.read( reinterpret_cast<char*>( header_.ptr() ), header_.size() );
#endif

   pe_PROFILING_SECTION {
      timeReadAll.end();
      sizeReadAll += header_.size();
   }

   // read magic number and file format version
   byte magicNumber1, magicNumber2, fileFormatVersionMajor, fileFormatVersionMinor;
   header_ >> magicNumber1 >> magicNumber2 >> fileFormatVersionMajor >> fileFormatVersionMinor;

   if( magicNumber1 != 'P' || magicNumber2 != 'E' )
      throw std::runtime_error( "Invalid file format." );

   if( fileFormatVersionMajor != 0 || fileFormatVersionMinor != 1 )
      throw std::runtime_error( "Unsupported file format version." );

   // read table of data type sizes
   byte sizes[5];
   for( int i = 0; i < 5; ++i ) {
      header_ >> sizes[i];
   }

   int fpSize = -1;
   {
      byte tmp = sizes[0];
      while( tmp != 0 ) {
         tmp >>= 1;
         ++fpSize;
      }
   }

   if( fpSize < 1 || ( 1 << fpSize ) != sizes[0] )
      throw std::runtime_error( "Invalid size of floating point data." );

   if( sizes[0] == sizeof( real ) ) {
      header_.setFloatingPointSize( 0 );
      buffer_.setFloatingPointSize( 0 );
      globals_.setFloatingPointSize( 0 );
   }
   else {
      header_.setFloatingPointSize( fpSize );
      buffer_.setFloatingPointSize( fpSize );
      globals_.setFloatingPointSize( fpSize );
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "FP size: 2^" << fpSize << " = " << (int)sizes[0] << "\n";
   }

   if( sizes[1] != sizeof( int ) )
      throw std::runtime_error( "Size of the int data type does not match the int data type used to produce the file." );

   if( sizes[2] != sizeof( size_t ) )
      throw std::runtime_error( "Size of the size_t data type does not match the size_t data type used to produce the file." );

   if( sizes[3] != sizeof( id_t ) )
      throw std::runtime_error( "Size of the id_t data type does not match the id_t data type used to produce the file." );

   if( sizes[4] != sizeof( bool ) )
      throw std::runtime_error( "Size of the bool data type does not match the bool data type used to produce the file." );

   uint32_t p;
   header_ >> p;
   uint32_t p0( MPISettings::size() );
   if( p0 != p && MPISettings::size() != 1 )
      throw std::runtime_error( "Number of processes differ from the number of processes used to produce the file." );

   // read rest of header since we now know the exact size
   header_.resize( ( 2 + p )*sizeof(uint32_t) );

   pe_PROFILING_SECTION {
      timeReadAll.start();
   }

#if HAVE_MPI
   MPI_File_read_all( fh, header_.ptr(), static_cast<int>( header_.size() ), MPI_BYTE, &status );
#else
   fh.read( reinterpret_cast<char*>( header_.ptr() ), header_.size() );
#endif

   pe_PROFILING_SECTION {
      timeReadAll.end();
      sizeReadAll += header_.size();
   }

   // read global body data offset
   uint32_t offsetGlobal;
   header_ >> offsetGlobal;

   // read processes' local body data offsets
   std::vector<uint32_t> offsets_( p + 1 );
   for( uint32_t i = 0; i < p + 1; ++i ) {
      header_ >> offsets_[i];
   }

   // read global body data
   globals_.resize( offsets_[0] - offsetGlobal );

   pe_PROFILING_SECTION {
      timeReadAll.start();
   }

#if HAVE_MPI
   MPI_File_read_at_all( fh, offsetGlobal, globals_.ptr(), static_cast<int>( globals_.size() ), MPI_BYTE, &status );
#else
   fh.seekg( offsetGlobal, std::ios::beg );
   fh.read( reinterpret_cast<char*>( globals_.ptr() ), globals_.size() );
#endif

   pe_PROFILING_SECTION {
      timeReadAll.end();
      sizeReadAll += globals_.size();
   }

   // clear all rigid bodies since we will reset the unique ID counter
   {
      WorldID world = theWorld();
      World::Iterator it = world->begin();
      while( it != world->end() )
         it = world->destroy( it );
   }

   // read local body data
   uint32_t offset, end;
   if( p0 == p ) {
      offset = offsets_[MPISettings::rank()];
      end = offsets_[MPISettings::rank() + 1];

      if( offset < offsetGlobal || offsets_[0] < offsetGlobal )
         throw std::runtime_error( "Invalid body data offset." );

      if( end < offset )
         throw std::runtime_error( "Invalid body data chunk size." );

      buffer_.resize( end - offset );

      pe_PROFILING_SECTION {
         timeReadLocal.start();
      }

#if HAVE_MPI
      MPI_File_read_at( fh, offset, buffer_.ptr(), static_cast<int>( buffer_.size() ), MPI_BYTE, &status );
      MPI_File_close( &fh );
#else
      fh.seekg( offset, std::ios::beg );
      fh.read( reinterpret_cast<char*>( buffer_.ptr() ), buffer_.size() );
#endif

      pe_PROFILING_SECTION {
         timeReadLocal.end();
         sizeReadLocal += buffer_.size();
      }

      // WARNING: the following code will not trigger exceptions. In debug mode assertions will be triggered. In release mode the behaviour is undefined if the file format is invalid.

      // restore the system ID counter
      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank " << MPISettings::rank() << " the old UniqueID<RigidBody>::counter_ is " << UniqueID<RigidBody>::counter_  << "\n";
      }
      unmarshal( buffer_, UniqueID<RigidBody>::counter_ );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank " << MPISettings::rank() << " the new UniqueID<RigidBody>::counter_ is " << UniqueID<RigidBody>::counter_  << "\n";
      }

      // unmarshal all process local bodies
      unmarshalAll( buffer_, false );

      // restore the system ID counter for global bodies
      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank " << MPISettings::rank() << " the old UniqueID<RigidBody>::globalCounter_ is " << UniqueID<RigidBody>::globalCounter_  << "\n";
      }
      unmarshal( globals_, UniqueID<RigidBody>::globalCounter_ );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank " << MPISettings::rank() << " the new UniqueID<RigidBody>::globalCounter_ is " << UniqueID<RigidBody>::globalCounter_  << "\n";
      }

      // unmarshal all global bodies
      unmarshalAll( globals_, true );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank " << MPISettings::rank() << " there are " << theWorld()->size() << " bodies in the world.\n";
      }
   }
   else {
      // reset system IDs
      UniqueID<RigidBody>::counter_ = UniqueID<RigidBody>::globalCounter_ = 0;

      for( uint32_t i = 0; i < p; ++i ) {
         offset = offsets_[i];
         end = offsets_[i + 1];

         if( offset < offsetGlobal || offsets_[0] < offsetGlobal )
            throw std::runtime_error( "Invalid body data offset." );

         if( end < offset )
            throw std::runtime_error( "Invalid body data chunk size." );

         buffer_.resize( end - offset );

         pe_PROFILING_SECTION {
            timeReadLocal.start();
         }

#if HAVE_MPI
         MPI_File_read_at( fh, offset, buffer_.ptr(), static_cast<int>( buffer_.size() ), MPI_BYTE, &status );
#else
         fh.seekg( offset, std::ios::beg );
         fh.read( reinterpret_cast<char*>( buffer_.ptr() ), buffer_.size() );
#endif

         pe_PROFILING_SECTION {
            timeReadLocal.end();
            sizeReadLocal += buffer_.size();
         }

         // skip the system ID counter
         id_t tmp;
         unmarshal( buffer_, tmp );

         // unmarshal all process local bodies
         unmarshalAll( buffer_, false, true );
      }

#if HAVE_MPI
      MPI_File_close( &fh );
#endif

      {
         // skip the system ID counter for global bodies
         id_t tmp;
         unmarshal( buffer_, tmp );

         // unmarshal all global bodies as local bodies
         unmarshalAll( globals_, true, true );
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank 0 there are " << theWorld()->size() << " bodies in the world.\n";
      }
   }

   pe_PROFILING_SECTION {
      timeAll.end();
      if( logging::loglevel >= logging::info ) {
         std::vector<timing::WcTimer*> timers;
         timers.push_back( &timeAll );
         timers.push_back( &timeOpen );
         timers.push_back( &timeReadAll );
         timers.push_back( &timeReadLocal );

         // Store the minimum time measurement of each timer over all time steps
         std::vector<double> minValues( timers.size() );
         for( std::size_t i = 0; i < minValues.size(); ++i )
            minValues[i] = timers[i]->min();

         // Store the maximum time measurement of each timer over all time steps
         std::vector<double> maxValues( timers.size() );
         for( std::size_t i = 0; i < maxValues.size(); ++i )
            maxValues[i] = timers[i]->max();

         // Store the total time measured of each timer over all time steps
         std::vector<double> totalValues( timers.size() );
         for( std::size_t i = 0; i < totalValues.size(); ++i )
            totalValues[i] = timers[i]->total();

         // Store the total number of measurements of each timer over all time steps
         std::vector<size_t> numValues( timers.size() );
         for( std::size_t i = 0; i < numValues.size(); ++i )
            numValues[i] = timers[i]->getCounter();

         pe_LOG_INFO_SECTION( log ) {
            log << "Timing results of BodyBinaryReader::readFile() on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min time     max time     avg time     total time   executions   bytes\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "total:                 "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << sizeReadAll + sizeReadLocal << "\n"
                << " - open                "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                << " - read all            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << sizeReadAll << "\n"
                << " - read local          "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << sizeReadLocal << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "Number of bodies created on current process: " << theWorld()->size() << "\n";
         }

         // Logging the profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );
            size_t             bodies( theWorld()->size() );

            // Reduce the minimum/maximum/total time measurement and number of measurements of each timer over all time steps and all ranks
            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &minValues[0],   static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxValues[0],   static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalValues[0], static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &numValues[0],   static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &bodies,         1,                                      MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &minValues[0],   0, static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &maxValues[0],   0, static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &totalValues[0], 0, static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &numValues[0],   0, static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &bodies,         0, 1,                                      MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }


            pe_ROOT_SECTION {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Timing results of BodyBinaryReader::readFile() reduced over all ranks:\n" << std::fixed << std::setprecision(4)
                           << "code part              min time     max time     avg time     total time   executions   bytes\n"
                           << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                           << "total:                 "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << offsets_.back() << "\n"
                           << " - open                "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                           << " - read all            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << offsets_.front() << "\n"
                           << " - read local          "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << offsets_.back() - offsets_.front() << "\n"
                           << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                           << "Number of bodies created on all processes: " << bodies << "\n";
               }
            }
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiates a process local union from given union parameter data.
 * \param objparam A union description in form of union parameter data.
 * \param global Indicates whether the union is global (true) or local (false).
 * \param reassignSystemID Reassigns new system IDs instead of trying to use the previous system IDs.
 * \return void
 *
 * Registration is postponed.
 */
UnionID BodyBinaryReader::instantiateUnion( const Union::Parameters& objparam, bool global, bool reassignSystemID ) {
   UnionID ret = 0;

   id_t objsid( objparam.sid_ );
   if( reassignSystemID )
      objsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();

   pe_INSTANTIATE_UNION( obj, objsid, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.m_, objparam.I_, objparam.q_, objparam.aabb_, objparam.visible_, objparam.fixed_, false ) {
      ret = obj;

      for( std::vector<Sphere::Parameters>::const_iterator it = objparam.spheres_.begin(); it != objparam.spheres_.end(); ++it ) {
         const Sphere::Parameters& subobjparam = *it;
         id_t subobjsid( subobjparam.sid_ );
         if( reassignSystemID )
            subobjsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
         SphereID subobj = instantiateSphere( subobjsid, subobjparam.uid_, subobjparam.gpos_, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( false );
         if( global ) subobj->setGlobal();
      }
      for( std::vector<Box::Parameters>::const_iterator it = objparam.boxes_.begin(); it != objparam.boxes_.end(); ++it ) {
         const Box::Parameters& subobjparam = *it;
         id_t subobjsid( subobjparam.sid_ );
         if( reassignSystemID )
            subobjsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
         BoxID subobj = instantiateBox( subobjsid, subobjparam.uid_, subobjparam.gpos_, subobjparam.rpos_, subobjparam.q_, subobjparam.lengths_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( false );
         if( global ) subobj->setGlobal();
      }
      for( std::vector<Capsule::Parameters>::const_iterator it = objparam.capsules_.begin(); it != objparam.capsules_.end(); ++it ) {
         const Capsule::Parameters& subobjparam = *it;
         id_t subobjsid( subobjparam.sid_ );
         if( reassignSystemID )
            subobjsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
         CapsuleID subobj = instantiateCapsule( subobjsid, subobjparam.uid_, subobjparam.gpos_, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( false );
         if( global ) subobj->setGlobal();
      }
      for( std::vector<Cylinder::Parameters>::const_iterator it = objparam.cylinders_.begin(); it != objparam.cylinders_.end(); ++it ) {
         const Cylinder::Parameters& subobjparam = *it;
         id_t subobjsid( subobjparam.sid_ );
         if( reassignSystemID )
            subobjsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
         CylinderID subobj = instantiateCylinder( subobjsid, subobjparam.uid_, subobjparam.gpos_, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( false );
         if( global ) subobj->setGlobal();
      }
      for( std::vector<Union::Parameters>::const_iterator it = objparam.unions_.begin(); it != objparam.unions_.end(); ++it ) {
         const Union::Parameters& subobjparam = *it;
         UnionID subobj = instantiateUnion( subobjparam, global, reassignSystemID );
         subobj->setRemote( false );
      }
      for( std::vector<Plane::Parameters>::const_iterator it = objparam.planes_.begin(); it != objparam.planes_.end(); ++it ) {
         const Plane::Parameters& subobjparam = *it;
         id_t subobjsid( subobjparam.sid_ );
         if( reassignSystemID )
            subobjsid = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
         PlaneID subobj = instantiatePlane( subobjsid, subobjparam.uid_, subobjparam.gpos_, subobjparam.rpos_, subobjparam.q_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( false );
         if( global ) subobj->setGlobal();
      }

      if( global ) obj->setGlobal();
   }

   return ret;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshals rigid bodies from the remaining buffer data.
 * \param buffer The buffer to extract rigid bodies from.
 * \param global Indicates whether the extracted bodies should be global (true) or local (false).
 * \param reassignSystemID Reassigns new system IDs instead of trying to use the previous system IDs.
 * \return void
 */
void BodyBinaryReader::unmarshalAll( Buffer& buffer, bool global, bool reassignSystemID ) {
   ManagerID manager( theDefaultManager() );

   while( !buffer.isEmpty() ) {
      GeomType geomType;
      uint32_t size;
      unmarshal( buffer, geomType );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "Unmarshaling geometry type " << (int)geomType << ".\n";
      }
      unmarshal( buffer, size );
      switch( geomType ) {
         case sphereType: {
            for( size_t i = 0; i < size; ++i ) {
               Sphere::Parameters objparam;
               unmarshal( buffer, objparam, false );
               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               SphereID obj = instantiateSphere( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.radius_, objparam.material_, objparam.visible_, objparam.fixed_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               if( global ) obj->setGlobal();
               manager->add( obj );
            }
            break;
         }
         case boxType: {
            for( size_t i = 0; i < size; ++i ) {
               Box::Parameters objparam;
               unmarshal( buffer, objparam, false );
               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               BoxID obj = instantiateBox( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.lengths_, objparam.material_, objparam.visible_, objparam.fixed_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               if( global ) obj->setGlobal();
               manager->add( obj );
            }
            break;
         }
         case capsuleType: {
            for( size_t i = 0; i < size; ++i ) {
               Capsule::Parameters objparam;
               unmarshal( buffer, objparam, false );
               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               CapsuleID obj = instantiateCapsule( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.radius_, objparam.length_, objparam.material_, objparam.visible_, objparam.fixed_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               if( global ) obj->setGlobal();
               manager->add( obj );
            }
            break;
         }
         case cylinderType: {
            for( size_t i = 0; i < size; ++i ) {
               Cylinder::Parameters objparam;
               unmarshal( buffer, objparam, false );
               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               CylinderID obj = instantiateCylinder( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.radius_, objparam.length_, objparam.material_, objparam.visible_, objparam.fixed_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               if( global ) obj->setGlobal();
               manager->add( obj );
            }
            break;
         }
         case triangleMeshType: {
            for( size_t i = 0; i < size; ++i ) {
               TriangleMesh::Parameters objparam;
               unmarshal( buffer, objparam, false );
               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               TriangleMeshID obj = instantiateTriangleMesh( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.vertices_, objparam.faceIndices_, objparam.material_, objparam.visible_, objparam.fixed_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               manager->add( obj );
            }
            break;
         }
         case unionType: {
            for( size_t i = 0; i < size; ++i ) {
               Union::Parameters objparam;
               unmarshal( buffer, objparam, false );
               UnionID obj = instantiateUnion( objparam, global, reassignSystemID );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               manager->add( obj );
            }
            break;
         }
         case planeType: {
            for( size_t i = 0; i < size; ++i ) {
               Plane::Parameters objparam;
               unmarshal( buffer, objparam, false );

               pe_INTERNAL_ASSERT( objparam.fixed_ == true, "Cannot instantiate unfixed planes." );

               if( reassignSystemID )
                  objparam.sid_ = global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create();
               PlaneID obj = instantiatePlane( objparam.sid_, objparam.uid_, objparam.gpos_, objparam.rpos_, objparam.q_, objparam.material_, objparam.visible_, false );
               obj->setLinearVel( objparam.v_ );
               obj->setAngularVel( objparam.w_ );
               obj->setRemote( false );
               if( global ) obj->setGlobal();
               manager->add( obj );
            }
            break;
         }
         default:
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Encountered invalid geometry type " << (int)geomType << ".\n";
            }
            throw std::runtime_error( "Unknown geometry type" );
            break;
      }
   }
}
//*************************************************************************************************

} // namespace pe
