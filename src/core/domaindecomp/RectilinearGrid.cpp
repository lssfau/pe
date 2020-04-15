//=================================================================================================
/*!
 *  \file src/core/domaindecomp/RectilinearGrid.cpp
 *  \brief Source file for the RectilinearGrid class
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

#if HAVE_MPI


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Decomposes the domain into a grid.
 *
 * \param origin The origin of the grid.
 * \param delta_x The length of each subdomain in the x dimension.
 * \param delta_y The length of each subdomain in the y dimension.
 * \param delta_z The length of each subdomain in the z dimension.
 * \param bc_lowerbound Specify the boundary conditions at the lower bounds of the grid.
 * \param bc_upperbound Specify the boundary conditions at the upper bounds of the grid.
 *
 * A previous decomposition is removed first. Boundary conditions can be boundaryConditionOutflow,
 * boundaryConditionOpen and boundaryConditionPeriodic. If periodic boundary conditions are
 * selected at a lower bound then it must also be selected at the corresponding upper bound.
 * Outflow boundary conditions are not supported by all collision systems and it is the users task
 * to prevent the outflow (e.g. by creating some restricting planes) if the collision system does
 * not support outflow. An open boundary condition extends the domain infinitely in the affected
 * dimension and associates the subdomain to the MPI process at the affected boundary.
 */
void RectilinearGrid::connect( const Vec3& origin, const std::vector<real>& delta_x, const std::vector<real>& delta_y, const std::vector<real>& delta_z, const Vector3<BoundaryCondition>& bc_lowerbound, const Vector3<BoundaryCondition>& bc_upperbound )
{
   const MPISystemID mpisystem( theMPISystem() );
   const int px( static_cast<int>( delta_x.size() ) );
   const int py( static_cast<int>( delta_y.size() ) );
   const int pz( static_cast<int>( delta_z.size() ) );
   int       p;

   MPI_Comm_size( MPI_COMM_WORLD, &p );
   if( p != px*py*pz )
      throw std::invalid_argument( "Number of grid cells does not match the number of MPI processes." );

   // Disconnect if a grid was instated before
   disconnect();

   bc_lowerbound_ = bc_lowerbound;
   bc_upperbound_ = bc_upperbound;

   // Remove previous domain decompositions if no grid was instated before
   while( mpisystem->begin() != mpisystem->end() )
      pe::disconnect( mpisystem->begin()->getRank() );

   int      dims   [] = { px, py, pz };
   int      periods[] = { bc_lowerbound[0] == boundaryConditionPeriodic, bc_lowerbound[1] == boundaryConditionPeriodic, bc_lowerbound[2] == boundaryConditionPeriodic };

   MPI_Cart_create( MPI_COMM_WORLD, 3, dims, periods, true, &cartcomm_ );
   mpisystem->setComm( cartcomm_ );
   MPI_Comm_rank( cartcomm_, &rank_ );
   MPI_Cart_coords( cartcomm_, rank_, 3, coords_ );

   // Calculate cell offsets
   x0_.resize( delta_x.size() + 1 );
   y0_.resize( delta_y.size() + 1 );
   z0_.resize( delta_z.size() + 1 );
   x0_[0] = origin[0];
   y0_[0] = origin[1];
   z0_[0] = origin[2];

   for( size_t i = 1; i <= delta_x.size(); ++i ) {
      x0_[i] = x0_[i-1] + delta_x[i-1];
      if( delta_x[i-1] <= 0 )
         throw std::invalid_argument( "Cell sizes must be positive." );
   }

   for( size_t i = 1; i <= delta_y.size(); ++i ) {
      y0_[i] = y0_[i-1] + delta_y[i-1];
      if( delta_y[i-1] <= 0 )
         throw std::invalid_argument( "Cell sizes must be positive." );
   }

   for( size_t i = 1; i <= delta_z.size(); ++i ) {
      z0_[i] = z0_[i-1] + delta_z[i-1];
      if( delta_z[i-1] <= 0 )
         throw std::invalid_argument( "Cell sizes must be positive." );
   }

   // Modify cell offsets for open boundary conditions
   pe_USER_ASSERT( bc_lowerbound[0] == bc_upperbound[0] || ( bc_lowerbound[0] != boundaryConditionPeriodic && bc_upperbound[0] != boundaryConditionPeriodic ), "Periodic boundary condition must be applied on both sides." );
   pe_USER_ASSERT( bc_lowerbound[1] == bc_upperbound[1] || ( bc_lowerbound[1] != boundaryConditionPeriodic && bc_upperbound[1] != boundaryConditionPeriodic ), "Periodic boundary condition must be applied on both sides." );
   pe_USER_ASSERT( bc_lowerbound[2] == bc_upperbound[2] || ( bc_lowerbound[2] != boundaryConditionPeriodic && bc_upperbound[2] != boundaryConditionPeriodic ), "Periodic boundary condition must be applied on both sides." );

   if( bc_lowerbound[0] == boundaryConditionOpen )
      x0_.front() = -inf;
   if( bc_upperbound[0] == boundaryConditionOpen )
      x0_.back()  =  inf;
   if( bc_lowerbound[1] == boundaryConditionOpen )
      y0_.front() = -inf;
   if( bc_upperbound[1] == boundaryConditionOpen )
      y0_.back()  =  inf;
   if( bc_lowerbound[2] == boundaryConditionOpen )
      z0_.front() = -inf;
   if( bc_upperbound[2] == boundaryConditionOpen )
      z0_.back()  =  inf;

   // Specify local subdomain. The local subdomain description must be exact (i.e. not a superset).
   real bounds_x[2] = {x0_[coords_[0]], x0_[coords_[0] + 1]};
   real bounds_y[2] = {y0_[coords_[1]], y0_[coords_[1] + 1]};
   real bounds_z[2] = {z0_[coords_[2]], z0_[coords_[2] + 1]};
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Rank " << rank_ << " at position (" << coords_[0] << ", " << coords_[1] << ", " << coords_[2] << ") takes up subdomain " << rangestr( bounds_x[0], bounds_x[1] ) << "x" << rangestr( bounds_y[0], bounds_y[1] ) << "x" << rangestr( bounds_z[0], bounds_z[1] ) << ".\n";
   }

   defineLocalDomain( intersect( intersect(
         HalfSpace( Vec3( +1,  0,  0 ), +bounds_x[0] ),
         HalfSpace( Vec3( -1,  0,  0 ), -bounds_x[1] ),
         HalfSpace( Vec3(  0, +1,  0 ), +bounds_y[0] ) ),
         HalfSpace( Vec3(  0, -1,  0 ), -bounds_y[1] ),
         HalfSpace( Vec3(  0,  0, +1 ), +bounds_z[0] ),
         HalfSpace( Vec3(  0,  0, -1 ), -bounds_z[1] )
      ) );

   // Connect remote subdomains of all 8 nearest neighbors. All subdomain descriptions must be disjoint and must be supersets of the actual subdomains.
   for( int dx = -1; dx <= 1; ++dx ) {
      for( int dy = -1; dy <= 1; ++dy ) {
         for( int dz = -1; dz <= 1; ++dz ) {
            if( dx == 0 && dy == 0 && dz == 0 )
               continue;

            int rank_nb, coords_nb[3] = {coords_[0] + dx, coords_[1] + dy, coords_[2] + dz};

            Vec3 offset;
            if( coords_nb[0] < 0 || coords_nb[0] >= px ) {
               if( bc_lowerbound[0] == boundaryConditionPeriodic )
                  offset[0] = -dx * ( x0_.back() - x0_.front() );
               else
                  continue;
            }
            if( coords_nb[1] < 0 || coords_nb[1] >= py ) {
               if( bc_lowerbound[1] == boundaryConditionPeriodic )
                  offset[1] = -dy * ( y0_.back() - y0_.front() );
               else
                  continue;
            }
            if( coords_nb[2] < 0 || coords_nb[2] >= pz ) {
               if( bc_lowerbound[2] == boundaryConditionPeriodic )
                  offset[2] = -dz * ( z0_.back() - z0_.front() );
               else
                  continue;
            }

            MPI_Cart_rank( cartcomm_, coords_nb, &rank_nb );

            if( dz == 0 ) {
               if( dx != 0 && dy != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << halfrangestr( dx, bounds_x[(dx + 1) / 2] * dx ) << "x" << halfrangestr( dy, bounds_y[(dy + 1) / 2] * dy ) << "x" << rangestr( bounds_z[0], bounds_z[1] ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real(dx), real( 0), real(0) ),  bounds_x[(dx + 1) / 2] * real(dx) ),
                        HalfSpace( Vec3( real( 0), real(dy), real(0) ),  bounds_y[(dy + 1) / 2] * real(dy) ),
                        HalfSpace( Vec3( real( 0), real( 0), real(+1) ), +bounds_z[0]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(-1) ), -bounds_z[1]                      )
                     ), offset );
               }
               else if ( dx != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << halfrangestr( dx, bounds_x[(dx + 1) / 2] * dx ) << "x" << rangestr( bounds_y[0], bounds_y[1] ) << "x" << rangestr( bounds_z[0], bounds_z[1] ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real(dx), real( 0), real( 0) ), bounds_x[(dx + 1) / 2] * real(dx) ),
                        HalfSpace( Vec3( real( 0), real(+1), real( 0)),  +bounds_y[0]                      ),
                        HalfSpace( Vec3( real( 0), real(-1), real( 0)),  -bounds_y[1]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(+1)),  +bounds_z[0]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(-1)),  -bounds_z[1]                      )
                     ), offset );
               }
               else if ( dy != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << rangestr( bounds_x[0], bounds_x[1] ) << "x" << halfrangestr( dy, bounds_y[(dy + 1) / 2] * dy ) << "x" << rangestr( bounds_z[0], bounds_z[1] ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real( 0), real(dy), real( 0) ), bounds_y[(dy + 1) / 2] * real(dy) ),
                        HalfSpace( Vec3( real(+1), real( 0), real( 0) ), +bounds_x[0]                      ),
                        HalfSpace( Vec3( real(-1), real( 0), real( 0) ), -bounds_x[1]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(+1) ), +bounds_z[0]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(-1) ), -bounds_z[1]                      )
                     ), offset );
               }
               else {
                  pe_INTERNAL_ASSERT( false, "Cannot connect to self." );
               }
            }
            else {
               if( dx != 0 && dy != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << halfrangestr( dx, bounds_x[(dx + 1) / 2] * dx ) << "x" << halfrangestr( dy, bounds_y[(dy + 1) / 2] * dy ) << "x" << halfrangestr( dz, bounds_z[(dz + 1) / 2] * dz ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real(dx), real( 0), real( 0) ), bounds_x[(dx + 1) / 2] * real(dx) ),
                        HalfSpace( Vec3( real( 0), real(dy), real( 0) ), bounds_y[(dy + 1) / 2] * real(dy) ),
                        HalfSpace( Vec3( real( 0), real( 0), real(dz) ), bounds_z[(dz + 1) / 2] * real(dz) )
                     ), offset );
               }
               else if ( dx != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << halfrangestr( dx, bounds_x[(dx + 1) / 2] * dx ) << "x" << rangestr( bounds_y[0], bounds_y[1] ) << "x" << halfrangestr( dz, bounds_z[(dz + 1) / 2] * dz ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real(dx), real( 0), real( 0) ), bounds_x[(dx + 1) / 2] * real(dx) ),
                        HalfSpace( Vec3( real( 0), real(+1), real( 0) ), +bounds_y[0]                      ),
                        HalfSpace( Vec3( real( 0), real(-1), real( 0) ), -bounds_y[1]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(dz) ), bounds_z[(dz + 1) / 2] * real(dz) )
                     ), offset );
               }
               else if ( dy != 0 ) {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << rangestr( bounds_x[0], bounds_x[1] ) << "x" << halfrangestr( dy, bounds_y[(dy + 1) / 2] * dy ) << "x" << halfrangestr( dz, bounds_z[(dz + 1) / 2] * dz ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real( 0), real(dy), real( 0) ), bounds_y[(dy + 1) / 2] * real(dy) ),
                        HalfSpace( Vec3( real(+1), real( 0), real( 0) ), +bounds_x[0]                      ),
                        HalfSpace( Vec3( real(-1), real( 0), real( 0) ), -bounds_x[1]                      ),
                        HalfSpace( Vec3( real( 0), real( 0), real(dz) ), bounds_z[(dz + 1) / 2] * real(dz) )
                     ), offset );
               }
               else {
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Connecting rank " << rank_ << " with rank " << rank_nb << " at relative position (" << dx << ", " << dy << ", " << dz << ") taking up subdomain " << rangestr( bounds_x[0], bounds_x[1] ) << "x" << rangestr( bounds_y[0], bounds_y[1] ) << "x" << halfrangestr( dz, bounds_z[(dz + 1) / 2] * dz ) << " with offset " << offset << ".\n";
                  }
                  pe::connect( rank_nb, intersect(
                        HalfSpace( Vec3( real(+1), ( 0), real( 0) ), +bounds_x[0] ),
                        HalfSpace( Vec3( real(-1), ( 0), real( 0) ), -bounds_x[1] ),
                        HalfSpace( Vec3( real( 0), (+1), real( 0) ), +bounds_y[0] ),
                        HalfSpace( Vec3( real( 0), (-1), real( 0) ), -bounds_y[1] ),
                        HalfSpace( Vec3( real( 0), ( 0), real(dz) ), bounds_z[(dz + 1) / 2] * real(dz) )
                     ), offset );
               }
            }
         }
      }
   }

#ifndef NDEBUG
   // Checking the process setup.
   theMPISystem()->checkProcesses();
#endif

   connected_ = true;
}
//*************************************************************************************************


} // namespace

#endif
