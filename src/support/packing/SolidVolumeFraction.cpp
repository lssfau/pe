//=================================================================================================
/*!
 *  \file src/support/packing/SolidVolumeFraction.cpp
 *  \brief Source file for the solid volume fraction helper functions.
 *
 *  Copyright (C) 2016 Tobias Preclik
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
// Includes
//*************************************************************************************************

#include <pe/support/packing/SolidVolumeFraction.h>
#include <pe/support/packing/Octree.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/collisionsystem/BodyShadowCopyStorageRetriever.h>
#include <pe/core/World.h>
#include <pe/core/MPISystem.h>


namespace pe {

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
SolidVolumeBoundingBox determineSolidVolumeBoundingBox()
{
   SolidVolumeBoundingBox solidVolumeBoundingBox(Vec3(std::numeric_limits<real>::infinity(), std::numeric_limits<real>::infinity(), std::numeric_limits<real>::infinity()), Vec3(-std::numeric_limits<real>::infinity(), -std::numeric_limits<real>::infinity(), -std::numeric_limits<real>::infinity()));

   CollisionSystemID cs( theCollisionSystem() );

   const BodyStorage<Config>& bodies = cs->getBodyStorage();
   for( BodyStorage<Config>::ConstIterator it = bodies.begin(); it != bodies.end(); ++it )
   {
      if( (*it)->isFixed() || (*it)->isGlobal() )
         continue;

      solidVolumeBoundingBox.unify((*it)->getAABB());
   }

   BodyShadowCopyStorageRetriever<Config> bodyShadowCopyStorageRetriever;
   const BodyStorage<Config>& shadowCopies = bodyShadowCopyStorageRetriever.retrieve();
   for( BodyStorage<Config>::ConstIterator it = shadowCopies.begin(); it != shadowCopies.end(); ++it )
   {
      if( (*it)->isFixed() || (*it)->isGlobal() )
         continue;

      solidVolumeBoundingBox.unify((*it)->getAABB());
   }

   real values[] = {solidVolumeBoundingBox.lbound()[0], solidVolumeBoundingBox.lbound()[1], solidVolumeBoundingBox.lbound()[2], -solidVolumeBoundingBox.ubound()[0], -solidVolumeBoundingBox.ubound()[1], -solidVolumeBoundingBox.ubound()[2]};

#if HAVE_MPI
   pe_MPI_SECTION {
      MPI_Allreduce( MPI_IN_PLACE, values, 6, MPITrait<real>::getType(), MPI_MIN, MPISettings::comm() );
   }
#endif

   return SolidVolumeBoundingBox(Vec3(values[0], values[1], values[2]), Vec3(-values[3], -values[4], -values[5]));
}
//*************************************************************************************************


//*************************************************************************************************
real determineLocalSolidVolumeFraction( SolidVolumeBoundingBox solidVolumeBoundingBox, real maxSamplingDistance )
{
   real solidVolumeFraction = 0.0;

   if( !solidVolumeBoundingBox.empty() )
   {
      std::size_t nx = static_cast<std::size_t>( std::ceil( ( solidVolumeBoundingBox.ubound()[0] - solidVolumeBoundingBox.lbound()[0] ) / maxSamplingDistance ) );
      std::size_t ny = static_cast<std::size_t>( std::ceil( ( solidVolumeBoundingBox.ubound()[1] - solidVolumeBoundingBox.lbound()[1] ) / maxSamplingDistance ) );
      std::size_t nz = static_cast<std::size_t>( std::ceil( ( solidVolumeBoundingBox.ubound()[2] - solidVolumeBoundingBox.lbound()[2] ) / maxSamplingDistance ) );

      real dx = ( solidVolumeBoundingBox.ubound()[0] - solidVolumeBoundingBox.lbound()[0] ) / nx;
      real dy = ( solidVolumeBoundingBox.ubound()[1] - solidVolumeBoundingBox.lbound()[1] ) / ny;
      real dz = ( solidVolumeBoundingBox.ubound()[2] - solidVolumeBoundingBox.lbound()[2] ) / nz;

      std::size_t solidSamples = 0;

      CollisionSystemID cs( theCollisionSystem() );

      const BodyStorage<Config>& bodies = cs->getBodyStorage();

      BodyShadowCopyStorageRetriever<Config> bodyShadowCopyStorageRetriever;
      const BodyStorage<Config>& shadowCopies = bodyShadowCopyStorageRetriever.retrieve();

      std::vector<ConstBodyID> allBodies;
      allBodies.reserve( bodies.size() + shadowCopies.size() );

      for( BodyStorage<Config>::ConstIterator body = bodies.begin(); body != bodies.end(); ++body )
         allBodies.push_back( *body );
      for( BodyStorage<Config>::ConstIterator body = shadowCopies.begin(); body != shadowCopies.end(); ++body )
         allBodies.push_back( *body );

      Octree octree( allBodies, 5 );

      for( std::size_t ix = 0; ix < nx; ++ix )
         for( std::size_t iy = 0; iy < ny; ++iy )
            for( std::size_t iz = 0; iz < nz; ++iz )
            {
               Vec3 p( solidVolumeBoundingBox.lbound()[0] + 0.5 * dx + ix * dx, solidVolumeBoundingBox.lbound()[1] + 0.5 * dy + iy * dy, solidVolumeBoundingBox.lbound()[2] + 0.5 * dz + iz * dz );

               if( octree.query( p ) )
                  ++solidSamples;
            }

      solidVolumeFraction = static_cast<real>( solidSamples ) / ( nx * ny * nz );
   }

   return solidVolumeFraction;
}
//*************************************************************************************************


//*************************************************************************************************
#if HAVE_MPI
real determineSolidVolumeFraction( SolidVolumeBoundingBox solidVolumeBoundingBox, const RectilinearGrid& grid, real maxSamplingDistance )
{
   // Intersect with local domain.
   solidVolumeBoundingBox.intersect( grid.getCell() );

   real volume = solidVolumeBoundingBox.volume();
   real solidVolumeFraction = determineLocalSolidVolumeFraction( solidVolumeBoundingBox, maxSamplingDistance );

   int rank = MPISettings::rank();
   int size = MPISettings::size();

   std::vector<real> volumes;
   if (rank == 0)
      volumes.resize(size);

   MPI_Gather( &volume, 1, MPITrait<real>::getType(), rank == 0 ? &volumes[0] : 0, 1, MPITrait<real>::getType(), 0, MPISettings::comm() );

   std::vector<real> solidVolumeFractions;
   if (rank == 0)
      solidVolumeFractions.resize(size);

   MPI_Gather( &solidVolumeFraction, 1, MPITrait<real>::getType(), rank == 0 ? &solidVolumeFractions[0] : 0, 1, MPITrait<real>::getType(), 0, MPISettings::comm() );

   if( rank == 0 )
   {
      real totalVolume = 0.0;
      for( std::size_t i = 0; i < static_cast<std::size_t>( size ); ++i )
         totalVolume += volumes[i];

      real totalSolidVolumeFraction = 0.0;
      for( std::size_t i = 0; i < static_cast<std::size_t>( size ); ++i )
      {
         totalSolidVolumeFraction += solidVolumeFractions[i] * ( volumes[i] / totalVolume );
      }

      MPI_Scatter( &totalSolidVolumeFraction, 1, MPITrait<real>::getType(), &solidVolumeFraction, 1, MPITrait<real>::getType(), 0, MPISettings::comm() );
   }
   else
   {
      MPI_Scatter( 0, 1, MPITrait<real>::getType(), &solidVolumeFraction, 1, MPITrait<real>::getType(), 0, MPISettings::comm() );
   }

   return solidVolumeFraction;
}
#endif
//*************************************************************************************************

} // namespace
