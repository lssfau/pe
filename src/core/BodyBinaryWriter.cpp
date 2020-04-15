//=================================================================================================
/*!
 *  \file src/core/BodyBinaryWriter.cpp
 *  \brief Writer for rigid body binary parameter files
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

#include <pe/core/BodyBinaryWriter.h>
#include <pe/core/Marshalling.h>
#include <pe/core/MPITrait.h>


namespace pe {

//*************************************************************************************************
/*!\brief Writes out a binary rigid body parameter file and returns immediately.
 * \param filename The filename of the parameter file to write to.
 * \return void
 */
void BodyBinaryWriter::writeFileAsync( const char* filename ) {
   timing::WcTimer timeAll, timeWait, timeExscan, timeOpen, timeGather, timeWrite;
   size_t sizeWrite( 0 ), bodies( 0 ), sizeWriteAll( 0 );

   using boost::numeric_cast;

   pe_PROFILING_SECTION {
      timeAll.start();
      timeWait.start();
   }

   // wait until we can reuse buffers
   wait();

   pe_PROFILING_SECTION {
      timeWait.end();
   }

   buffer_.clear();
   header_.clear();
   globals_.clear();

   buffer_.setFloatingPointSize( fpSize_ );
   header_.setFloatingPointSize( fpSize_ );
   globals_.setFloatingPointSize( fpSize_ );

   ConstWorldID world = theWorld();

   marshal( buffer_, UniqueID<RigidBody>::counter_ );
   bodies += marshalAllPrimitives<Sphere>      ( buffer_, world );
   bodies += marshalAllPrimitives<Box>         ( buffer_, world );
   bodies += marshalAllPrimitives<Capsule>     ( buffer_, world );
   bodies += marshalAllPrimitives<Cylinder>    ( buffer_, world );
   //bodies += marshalAllPrimitives<Plane>       ( buffer_, world );
   bodies += marshalAllPrimitives<TriangleMesh>( buffer_, world );
   bodies += marshalAllPrimitives<Union>       ( buffer_, world );

   size_t localSize = buffer_.size(), headerSize = 0;

   pe_EXCLUSIVE_SECTION( 0 ) {
      // marshal global bodies
      marshal( globals_, UniqueID<RigidBody>::globalCounter_ );
      bodies += marshalAllPrimitives<Sphere>      ( globals_, world, true );
      bodies += marshalAllPrimitives<Box>         ( globals_, world, true );
      bodies += marshalAllPrimitives<Capsule>     ( globals_, world, true );
      bodies += marshalAllPrimitives<Cylinder>    ( globals_, world, true );
      bodies += marshalAllPrimitives<Plane>       ( globals_, world, true );
      bodies += marshalAllPrimitives<TriangleMesh>( globals_, world, true );
      bodies += marshalAllPrimitives<Union>       ( globals_, world, true );

      // write header
      const byte fileFormatVersionMajor( 0 );
      const byte fileFormatVersionMinor( 1 );
      header_ << static_cast<byte>('P') << static_cast<byte>('E') << fileFormatVersionMajor << fileFormatVersionMinor;

      // table of data type sizes
      if( fpSize_ == 0 )
         header_ << static_cast<byte>( sizeof(real) );
      else
         header_ << static_cast<byte>( 1 << fpSize_ );
         
      header_ << static_cast<byte>( sizeof(int) )
              << static_cast<byte>( sizeof(size_t) )
              << static_cast<byte>( sizeof(id_t) )
              << static_cast<byte>( sizeof(bool) );

      header_ << numeric_cast<uint32_t>( MPISettings::size() );

      headerSize = header_.size() + ( MPISettings::size() + 2 ) * sizeof( uint32_t );
      localSize = headerSize + globals_.size() + buffer_.size();
   }

   // determine offset of chunk for local body descriptions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " size of local bodies chunk is " << localSize << "\n";
   }

   pe_PROFILING_SECTION {
      timeExscan.start();
   }

   size_t offset;
#if HAVE_MPI
   MPI_Exscan( &localSize, &offset, 1, MPITrait<size_t>::getType(), MPI_SUM, MPISettings::comm() );
#else
   // to silence warnings
   offset = headerSize + globals_.size();
#endif

   pe_PROFILING_SECTION {
      timeExscan.end();
   }

   pe_EXCLUSIVE_SECTION( 0 ) {
      offset = headerSize + globals_.size();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " offset of local bodies chunk is " << offset << "\n";
   }

   pe_PROFILING_SECTION {
      timeOpen.start();
   }

   std::string filenameCopy( filename );
#if HAVE_MPI
   MPI_File_open( MPISettings::comm(), &filenameCopy[0], MPI_MODE_WRONLY | MPI_MODE_CREATE, MPI_INFO_NULL, &fh_ );
#else
   fh_.open( &filenameCopy[0], std::ofstream::binary );
#endif
   fhOpen_ = true;

   pe_PROFILING_SECTION {
      timeOpen.end();
      timeWrite.start();
   }

   // flush local bodies chunk to file
#if HAVE_MPI
   MPI_Request request;
   MPI_File_iwrite_at( fh_, offset, buffer_.ptr(), static_cast<int>( buffer_.size() ), MPI_BYTE, &request );
   requests_.push_back( request );
#else
   fh_.seekp( offset );
   fh_.write( reinterpret_cast<const char*>( buffer_.ptr() ), buffer_.size() );
#endif
   size_t end = offset + buffer_.size();

   pe_PROFILING_SECTION {
      sizeWrite += buffer_.size();
      timeGather.start();
   }

   if( MPISettings::rank() == 0 ) {
      // create and write table of processes' local and global body data offsets into header
      std::vector<size_t> offsets( MPISettings::size() );
#if HAVE_MPI
      MPI_Gather( &end, 1, MPITrait<size_t>::getType(), &offsets[0], 1, MPITrait<size_t>::getType(), 0, MPISettings::comm() );
#else
      offsets[0] = end;
#endif

      pe_PROFILING_SECTION {
         timeGather.end();
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank 0 offset of global bodies chunk is " << headerSize << "\n";
      }
      header_ << numeric_cast<uint32_t>( headerSize );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "On rank 0 offset of local bodies chunk is " << offset << "\n";
      }
      header_ << numeric_cast<uint32_t>( offset );
      for( size_t i = 0; i < offsets.size(); ++i ) {
         header_ << numeric_cast<uint32_t>( offsets[i] );
         pe_LOG_DEBUG_SECTION( log ) {
            log << "On rank " << i << " end of local bodies chunk is " << offsets[i] << "\n";
         }
      }

      // flush header to file
#if HAVE_MPI
      MPI_File_iwrite_at( fh_, 0, header_.ptr(), static_cast<int>( header_.size() ), MPI_BYTE, &request );
      requests_.push_back( request );
#else
      fh_.seekp( 0 );
      fh_.write( reinterpret_cast<const char*>( header_.ptr() ), header_.size() );
#endif

      // flush global bodies chunk to file
#if HAVE_MPI
      MPI_File_iwrite_at( fh_, headerSize, globals_.ptr(), static_cast<int>( globals_.size() ), MPI_BYTE, &request );
      requests_.push_back( request );
#else
      fh_.seekp( headerSize );
      fh_.write( reinterpret_cast<const char*>( globals_.ptr() ), globals_.size() );
#endif

      pe_PROFILING_SECTION {
         sizeWrite += header_.size() + globals_.size();
         sizeWriteAll = offsets.back();
      }
   }
   else {
#if HAVE_MPI
      MPI_Gather( &end, 1, MPITrait<size_t>::getType(), 0, 0, MPITrait<size_t>::getType(), 0, MPISettings::comm() );
#endif

      pe_PROFILING_SECTION {
         timeGather.end();
      }
   }

   pe_PROFILING_SECTION {
      timeWrite.end();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "On rank " << MPISettings::rank() << " rigid body counter is " << UniqueID<RigidBody>::counter_ << "\n";
      log << "On rank " << MPISettings::rank() << " marshaled " << bodies << " rigid bodies out of " << theWorld()->size() << " in the world\n";
   }

   pe_PROFILING_SECTION {
      timeAll.end();
      if( logging::loglevel >= logging::info ) {
         std::vector<timing::WcTimer*> timers;
         timers.push_back( &timeAll );
         timers.push_back( &timeWait );
         timers.push_back( &timeExscan );
         timers.push_back( &timeOpen );
         timers.push_back( &timeWrite );
         timers.push_back( &timeGather );

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
            log << "Timing results of BodyBinaryWriter::writeFileAsync() on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min time     max time     avg time     total time   executions   bytes\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "total                  "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << sizeWrite << "\n"
                << "  wait                 "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                << "  exscan               "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << 0 << "\n"
                << "  open                 "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << 0 << "\n"
                << "  write                "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "   " << std::setw(10) << sizeWrite << "\n"
                << "    gather             "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "   " << std::setw(10) << 0 << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                << "Number of bodies marshaled on current process: " << bodies << " (out of " << theWorld()->size() << " in the world)\n";
         }

         // Logging the profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total time measurement and number of measurements of each timer over all time steps and all ranks
            if( rank == 0 ) {
               MPI_Reduce( MPI_IN_PLACE, &minValues[0],   static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxValues[0],   static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalValues[0], static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &numValues[0],   static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( MPI_IN_PLACE, &bodies,         1,                                      MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
            }
            else {
               MPI_Reduce( &minValues[0],   0, static_cast<int>( minValues.size() ),   MPITrait<double>::getType(), MPI_MIN, 0, comm );
               MPI_Reduce( &maxValues[0],   0, static_cast<int>( maxValues.size() ),   MPITrait<double>::getType(), MPI_MAX, 0, comm );
               MPI_Reduce( &totalValues[0], 0, static_cast<int>( totalValues.size() ), MPITrait<double>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( &numValues[0],   0, static_cast<int>( numValues.size() ),   MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
               MPI_Reduce( &bodies,         0, 1,                                      MPITrait<size_t>::getType(), MPI_SUM, 0, comm );
            }


            pe_EXCLUSIVE_SECTION( 0 ) {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Timing results of BodyBinaryWriter::writeFileAsync() reduced over all ranks:\n" << std::fixed << std::setprecision(4)
                      << "code part              min time     max time     avg time     total time   executions   bytes\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                      << "total:                 "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "   " << std::setw(10) << sizeWriteAll << "\n"
                      << "  wait                 "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "   " << std::setw(10) << 0 << "\n"
                      << "  exscan               "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "   " << std::setw(10) << 0 << "\n"
                      << "  open                 "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "   " << std::setw(10) << 0 << "\n"
                      << "  write                "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "   " << std::setw(10) << sizeWriteAll << "\n"
                      << "    gather             "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "   " << std::setw(10) << 0 << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------\n"
                      << "Number of bodies marshaled on all processes: " << bodies << "\n";
               }
            }
         }
      }
   }
}
//*************************************************************************************************

} // namespace pe
