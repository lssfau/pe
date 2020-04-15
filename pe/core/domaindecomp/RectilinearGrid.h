//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/RectilinearGrid.h
 *  \brief Header file for the RectilinearGrid class
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

#ifndef _PE_CORE_DOMAINDECOMP_RECTILINEARGRID_H_
#define _PE_CORE_DOMAINDECOMP_RECTILINEARGRID_H_

#if HAVE_MPI


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/detection/coarse/BoundingBox.h>
#include <pe/core/domaindecomp/BoundaryCondition.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/math/Vector3.h>
#include <vector>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Decomposes the domain into a 3D rectilinear grid.
 * \ingroup domaindecomp
 *
 * On construction MPI must be initialized. On destruction MPI should not be finalized for properly
 * freeing resources. \a connect sets up the domain decomposition and \a disconnect resets the
 * domain decomposition. All get functions are only valid to call if the rectilinear grid is
 * connected.
 */
class PE_PUBLIC RectilinearGrid
{
public:
   //**Type definitions****************************************************************************
   typedef pe::detection::coarse::BoundingBox<real> AABB;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit RectilinearGrid();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   ~RectilinearGrid();
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline Vector3<size_t> getCoords() const;
   inline Vector3<size_t> getCoords( int rank ) const;
   inline int             getRank() const;
   inline int             getRank( const Vector3<size_t>& coords ) const;
   inline AABB            getCell() const;
   inline AABB            getCell( const Vector3<size_t>& coords ) const;
   inline Vector3<BoundaryCondition> getBoundaryConditionLowerBound() const;
   inline Vector3<BoundaryCondition> getBoundaryConditionUpperBound() const;
   inline Vec3            getOrigin() const;
   inline Vec3            getLengths() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void connect( const Vec3& origin, const std::vector<real>& dx, const std::vector<real>& dy, const std::vector<real>& dz, const Vector3<BoundaryCondition>& bc_lowerbound, const Vector3<BoundaryCondition>& bc_upperbound );
   void connect( const Vec3& origin, const Vec3& lengths, const Vector3<size_t>& num, const Vector3<BoundaryCondition>& bc_lowerbound, const Vector3<BoundaryCondition>& bc_upperbound );
   void disconnect();
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*! \cond PE_INTERNAL */
   std::string halfrangestr( int normal, real offset );
   std::string rangestr( real lb, real ub );
   /*! \endcond */
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   MPI_Comm cartcomm_;
   int coords_[3];  //!< Coordinates of this MPI process in the cartesian communicator grid.
   int rank_;
   std::vector<real> x0_;
   std::vector<real> y0_;
   std::vector<real> z0_;
   Vector3<BoundaryCondition> bc_lowerbound_, bc_upperbound_;
   bool connected_;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructs the (disconnected) grid object.
 *
 * \exception std::runtime_error MPI system is not initialized.
 */
inline RectilinearGrid::RectilinearGrid()
   : connected_( false )
{
   int flagInit( 0 ), flagFinalize( 0 );
   MPI_Initialized( &flagInit );
   MPI_Finalized( &flagFinalize );

   if( !flagInit && !flagFinalize )
      throw std::runtime_error("MPI is not yet initialized or already finalized.");
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructs the grid object.
 *
 * If MPI is not yet finalized the grid decomposition is disconnected.
 */
inline RectilinearGrid::~RectilinearGrid()
{
   int flag( 0 );
   MPI_Finalized( &flag );

   if( !flag )
      disconnect();

   // WARNING: If a rectilinear grid is destroyed after MPI finalization then a (still reachable) memory leak occurs on exit because the MPI communicator is not freed.
}




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Returns the (integral) coordinates of the current process in the grid.
 *
 * \return The coordinates of the current process in the grid.
 *
 * The coordinates start at 0.
 */
//*************************************************************************************************
inline Vector3<size_t> RectilinearGrid::getCoords() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );
   return Vector3<size_t>( static_cast<size_t>( coords_[0] ), static_cast<size_t>( coords_[1] ), static_cast<size_t>( coords_[2] ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the (integral) coordinates of the specified process in the grid.
 * \param rank The rank of the process whose coordinates are to be returned.
 *
 * \return The coordinates of the specified process in the grid.
 */
//*************************************************************************************************
inline Vector3<size_t> RectilinearGrid::getCoords( int rank ) const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   int coords[3];
   MPI_Cart_coords( cartcomm_, rank, 3, coords );
   return Vector3<size_t>( static_cast<size_t>( coords[0] ), static_cast<size_t>( coords[1] ), static_cast<size_t>( coords[2] ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the rank of the current process.
 *
 * \return The rank of the current process.
 */
//*************************************************************************************************
inline int RectilinearGrid::getRank() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return rank_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the rank of the process identified by the given coordinates.
 * \param coords The coordinates of the process whose rank is to be returned.
 *
 * \return The rank of the process identified by the given coordinates.
 */
//*************************************************************************************************
inline int RectilinearGrid::getRank( const Vector3<size_t>& coords ) const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   int rank_nb, coords_nb[3] = { static_cast<int>(coords[0]), static_cast<int>(coords[1]), static_cast<int>(coords[2]) };
   MPI_Cart_rank( cartcomm_, coords_nb, &rank_nb );

   return rank_nb;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a description of the subdomain of the current process in form of a bounding box.
 *
 * \return A description of the subdomain of the current process in form of a bounding box.
 */
//*************************************************************************************************
inline RectilinearGrid::AABB RectilinearGrid::getCell() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return getCell( Vector3<size_t>( coords_[0], coords_[1], coords_[2] ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a description of the subdomain of the process identified by the given (integral) coordinates in form of a bounding box.
 * \param coords The (integral) coordinates of the process whose subdomain description is to be returned.
 *
 * \return A description of the subdomain of the process identified by the given coordinates in form of a bounding box.
 */
//*************************************************************************************************
inline RectilinearGrid::AABB RectilinearGrid::getCell( const Vector3<size_t>& coords ) const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return AABB( x0_[ coords[0] ], y0_[ coords[1] ], z0_[ coords[2] ], x0_[ coords[0] + 1 ], y0_[ coords[1] + 1 ], z0_[coords[2] + 1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a 3D vector with components identifying the boundary conditions at the each lower bound of the grid.
 *
 * \return A 3D vector with components identifying the boundary conditions at the each lower bound of the grid.
 */
//*************************************************************************************************
inline Vector3<BoundaryCondition> RectilinearGrid::getBoundaryConditionLowerBound() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return bc_lowerbound_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a 3D vector with components identifying the boundary conditions at the each upper bound of the grid.
 *
 * \return A 3D vector with components identifying the boundary conditions at the each upper bound of the grid.
 */
//*************************************************************************************************
inline Vector3<BoundaryCondition> RectilinearGrid::getBoundaryConditionUpperBound() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return bc_upperbound_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the origin of the grid.
 *
 * \return The origin of the grid.
 */
//*************************************************************************************************
inline Vec3 RectilinearGrid::getOrigin() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return Vec3( x0_.front(), y0_.front(), z0_.front() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the length of the grid in each dimension.
 *
 * \return The length of the grid in each dimension.
 */
//*************************************************************************************************
inline Vec3 RectilinearGrid::getLengths() const {
   pe_USER_ASSERT( connected_, "Grid is not yet connected." );

   return Vec3( x0_.back() - x0_.front(), y0_.back() - y0_.front(), z0_.back() - z0_.front() );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Decomposes the domain into a grid.
 *
 * \param origin The origin of the grid.
 * \param lengths The length of the grid in each dimension.
 * \param num The number of processes in each dimension.
 * \param bc_lowerbound Specify the boundary conditions at the lower bounds of the grid.
 * \param bc_upperbound Specify the boundary conditions at the upper bounds of the grid.
 *
 * A previous decomposition is removed first. This method is for convenience only. For further
 * informations refer to the other \a connect method.
 */
inline void RectilinearGrid::connect( const Vec3& origin, const Vec3& lengths, const Vector3<size_t>& num, const Vector3<BoundaryCondition>& bc_lowerbound, const Vector3<BoundaryCondition>& bc_upperbound )
{
   std::vector<real> delta_x( num[0], lengths[0]/num[0] ), delta_y( num[1], lengths[1]/num[1] ), delta_z( num[2], lengths[2]/num[2] );
   connect( origin, delta_x, delta_y, delta_z, bc_lowerbound, bc_upperbound );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Disconnects the grid.
 *
 * \return void
 *
 * If the grid decomposition is not instated the function silently returns.
 */
//*************************************************************************************************
inline void RectilinearGrid::disconnect() {
   if( !connected_ )
      return;

   MPISystemID mpisystem( theMPISystem() );

   if( mpisystem->getComm() != cartcomm_ ) {
      MPI_Comm_free( &cartcomm_ );
      connected_ = false;
      return;
   }

   // Remove domain decompositions
   while( mpisystem->begin() != mpisystem->end() )
      pe::disconnect( mpisystem->begin()->getRank() );

   // Reset domain
   defineLocalDomain( HalfSpace( Vec3(1, 0, 0), -inf ) );

   mpisystem->setComm( MPI_COMM_WORLD );
   MPI_Comm_free( &cartcomm_ );

   connected_ = false;
}
//*************************************************************************************************


//*************************************************************************************************
inline std::string RectilinearGrid::halfrangestr( int normal, real offset ) {
   std::ostringstream out;
   pe_USER_ASSERT( normal == 1 || normal == -1, "Unnormalized normal in range output." );
   if( normal > 0 )
      out << "[" << offset / normal << "; inf)";
   else
      out << "(-inf; " << offset / normal << "]";
   return out.str();
}
//*************************************************************************************************


//*************************************************************************************************
inline std::string RectilinearGrid::rangestr( real lb, real ub ) {
   std::ostringstream out;
   pe_USER_ASSERT( lb < ub, "Invalid reversed range." );
   out << "[" << lb << "; " << ub << "]";
   return out.str();
}
//*************************************************************************************************

} // namespace pe

#endif

#endif
