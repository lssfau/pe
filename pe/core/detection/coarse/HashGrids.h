//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/HashGrids.h
 *  \brief Implementation of the 'Hierarchical Hash Grids' coarse collision detection algorithm
 *
 *  Copyright (C) 2009 Klaus Iglberger
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

#ifndef _PE_CORE_DETECTION_COARSE_HASHGRIDS_H_
#define _PE_CORE_DETECTION_COARSE_HASHGRIDS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <list>
#include <vector>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>
#include <pe/system/HashGrids.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the 'Hierarchical Hash Grids' coarse collision detection algorithm.
 * \ingroup coarse_collision_detection
 *
 * The 'Hierarchical Hash Grids' coarse collision detection algorithm is based on a spatial
 * partitioning scheme that uses a hierarchy of uniform, hash storage based grids. Uniform grids
 * subdivide the simulation space into cubic cells. A hierarchy of spatially superimposed grids
 * provides a set of grids with each grid subdividing the very same space, however possessing
 * different and thus unique-sized cells. Spatial partitioning is achieved by assigning every
 * rigid body to exactly one cell in exactly one grid - that is the grid with the smallest cells
 * that are larger than the rigid body (more precisely cells that are larger than the longest
 * edge of the rigid body's axis-aligned bounding box). As a consequence, the collision detection
 * is reduced to only checking body's that are assigned to spatially directly adjacent cells.
 *
 * Key features of this implementation are not only an <b>average-case computational complexity
 * of order O(N)</b> as well as a space complexity of order O(N), but also a short actual runtime
 * combined with low memory consumption. Moreover, the data structure representing the hierarchy
 * of grids has the ability to, at runtime, automatically and perfectly adapt to the bodies of the
 * underlying simulation. Also, arbitrary bodies can be added and removed in constant time, O(1).
 * Consequently, the coarse collision detection algorithm based on these hierarchical hash grids
 * is <b>especially well-suited for large-scale simulations</b> involving very large numbers of
 * bodies.
 *
 * For further information and a much more detailed explanation of this algorithm see
 *     http://www10.informatik.uni-erlangen.de/Publications/Theses/2009/Schornbaum_SA09.pdf
 */
template< typename C >  // Type of the configuration
class HashGrids : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                              Config;        //!< Type of the configuration.
   typedef HashGrids<Config>              This;          //!< Type of this HashGrids instance.
   typedef BodyStorage<Config>            BS;            //!< Type of the body storage.
   typedef typename Config::BodyID        BodyID;        //!< Handle for a rigid body.
   typedef typename Config::FineDetector  FineDetector;  //!< Type of the fine collision detector.
   //**********************************************************************************************

private:
   //**Type definitions****************************************************************************
   //! Vector for storing (handles to) rigid bodies.
   typedef typename std::vector<BodyID>  BodyVector;
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Implementation of the hash grid data structure.
   */
   class HashGrid
   {
    private:
      //**Type definitions*************************************************************************
      //! The signed integer type that is used for storing offsets to neighboring cells. 
      typedef long offset_t;
      //*******************************************************************************************

      //*******************************************************************************************
      /*!\brief Data structure for representing a cell in the hash grid (used by the 'Hierarchical
      //        Hash Grids' coarse collision detection algorithm).
      */
      struct Cell
      {
         BodyVector* bodies_;           /*!< \brief The cell's body container that stores (handles to)
                                             all the bodies that are assigned to this cell. */
                                        /*!< Note that only a pointer to such a body container is
                                             stored: in order to save memory, this container object
                                             is only allocated as long as there are bodies assigned
                                             to this cell. */
         offset_t*   neighborOffset_;   /*!< \brief Pointer to an array that is storing offsets that
                                             can be used to directly access all the neighboring
                                             cells in the hash grid. */
         size_t      occupiedCellsId_;  //!< The cell's index in the \a occupiedCells_ vector.
      };
      //*******************************************************************************************

      //**Type definitions*************************************************************************
      //! Vector for storing pointers to (body-occupied) cells.
      typedef typename std::vector<Cell*>  CellVector;
      //*******************************************************************************************

    public:
      //**Constructor******************************************************************************
      /*!\name Constructor */
      //@{
      explicit HashGrid( real cellSpan );
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      /*!\name Destructor */
      //@{
      ~HashGrid();
      //@}
      //*******************************************************************************************

      //**Getter functions*************************************************************************
      /*!\name Getter functions */
      //@{
      real getCellSpan() const { return cellSpan_; }  //!< Getter for \a cellSpan_.
      //@}
      //*******************************************************************************************

      //**Add/remove functions*********************************************************************
      /*!\name Add/remove functions */
      //@{
      inline void add   ( BodyID body );
      inline void remove( BodyID body );
      //@}
      //*******************************************************************************************

      //**Utility functions************************************************************************
      /*!\name Utility functions */
      //@{
      void update( BodyID body );

      template< typename Contacts >
      size_t process      ( BodyID** gridBodies, Contacts& contacts ) const;

      template< typename Contacts >
      void   processBodies( BodyID* bodies, size_t bodyCount, Contacts& contacts ) const;

      void clear();
      //@}
      //*******************************************************************************************

    private:
      //**Utility functions************************************************************************
      /*!\name Utility functions */
      //@{
      void initializeNeighborOffsets();

      size_t hash( BodyID body ) const;

      void add   ( BodyID body, Cell* cell );
      void remove( BodyID body, Cell* cell );

      void enlarge();
      //@}
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      Cell* cell_;  //!< Linear array of cells representing the three-dimensional hash grid.

      size_t xCellCount_;  //!< Number of cells allocated in x-axis direction.
      size_t yCellCount_;  //!< Number of cells allocated in y-axis direction.
      size_t zCellCount_;  //!< Number of cells allocated in z-axis direction.

      size_t xHashMask_;  //!< Bit-mask required for the hash calculation in x-axis direction (\a xCellCount_ - 1).
      size_t yHashMask_;  //!< Bit-mask required for the hash calculation in y-axis direction (\a yCellCount_ - 1).
      size_t zHashMask_;  //!< Bit-mask required for the hash calculation in z-axis direction (\a zCellCount_ - 1).

      size_t xyCellCount_;   /*!< \brief Number of cells allocated in x-axis direction multiplied by
                                         the number of cells allocated in y-axis direction. */
      size_t xyzCellCount_;  //!< Total number of allocated cells.

      size_t enlargementThreshold_;  /*!< \brief The enlargement threshold - the moment the number
                                                 of assigned bodies exceeds this threshold, the
                                                 size of this hash grid is increased. */

      real cellSpan_;         //!< The grid's cell size (edge length of the underlying cubic grid cells).
      real inverseCellSpan_;  //!< The inverse cell size.
                              /*!< Required for replacing floating point divisions with multiplications
                                   during the hash computation. */

      CellVector occupiedCells_;  //!< A grid-global list that keeps track of all body-occupied cells.
                                  /*!< The list is required in order to avoid iterating through all
                                       grid cells whenever all bodies that are stored in the grid
                                       need to be addressed.  */

      size_t bodyCount_;  //!< Number of bodies assigned to this hash grid.

      offset_t stdNeighborOffset_[27];  /*!< \brief Array of offsets to the neighboring cells (valid
                                                    for all the inner cells of the hash grid). */
      //@}
      //*******************************************************************************************	  
   };
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   //! List for storing all the hash grids that are in use.
   /*! This data structure is used to represent the grid hierarchy. All hash grids are stored in
       ascending order by the size of their cells. */
   typedef typename std::list<HashGrid*>  GridList;
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   HashGrids( BS& bodystorage );
   HashGrids( BS& bodystorage, BS& bodystorageShadowCopies );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~HashGrids();
   //@}
   //**********************************************************************************************

   //**Add/remove functions************************************************************************
   /*!\name Add/remove functions */
   //@{
   inline void add   ( BodyID body );
          void remove( BodyID body );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Contacts > void findContacts( Contacts& contacts );
                                 void clear       ();
   //@}
   //**********************************************************************************************

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Contacts >
   static inline void collide( BodyID a, BodyID b, Contacts& contacts );
   //@}
   //**********************************************************************************************

private:
   //**Add functions*******************************************************************************
   /*!\name Add functions */
   //@{
   void addGrid( BodyID body );
   void addList( BodyID body );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static inline bool powerOfTwo( size_t number );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   BS&         bodystorage_;    //!< Reference to the central body storage.
   BS&         bodystorageShadowCopies_;    //!< Reference to the body storage containing body shadow copies.
   BodyVector  bodiesToAdd_;    //!< Vector of all bodies to be added to the HHG data structure.
                                /*!< This vector contains all bodies that were registered with
                                     the coarse collision detector and have not yet been added
                                     to the hierarchical hash grids data structure. */
   BodyVector  nonGridBodies_;  //!< Vector of all unassigned rigid bodies.
                                /*!< This vector contains all bodies that are not assigned to any
                                     grid in the hierarchy. A body is not assigned to any grid if
                                     the body is infinite in size or if the detection algorithm
                                     based on hierarchical hash grids has not yet been activated
                                     (see \a gridActive_). */
   GridList    gridList_;       //!< List of all grids that form the hierarchy.
                                /*!< The grids in this list are sorted in ascending order by the
                                     size of their cells. */
   bool        gridActive_;     //!< Grid activation flag.
                                /*!< This flag indicates whether the detection algorithm based on
                                     hierarchical hash grids is activated. If the simulation only
                                     consists of a very small number of bodies, each body is simply
                                     checked against every other body. This strategy proves to be
                                     faster than involving the far more complex mechanisms of the
                                     hierarchical hash grids. */
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( This, typename Config::CoarseDetector );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the HashGrid class.
 */
template< typename C >  // Type of the configuration
HashGrids<C>::HashGrid::HashGrid( real cellSpan )
{
   using namespace pe::detection::coarse::hhg;

   // Initialization of all member variables and ...
   xCellCount_   = powerOfTwo( xCellCount ) ? xCellCount : 16;
   yCellCount_   = powerOfTwo( yCellCount ) ? yCellCount : 16;
   zCellCount_   = powerOfTwo( zCellCount ) ? zCellCount : 16;

   xHashMask_    = xCellCount_ - 1;
   yHashMask_    = yCellCount_ - 1;
   zHashMask_    = zCellCount_ - 1;

   xyCellCount_  = xCellCount_ * yCellCount_;
   xyzCellCount_ = xyCellCount_ * zCellCount_;

   enlargementThreshold_ = xyzCellCount_ / minimalGridDensity;

   // ... allocation of the linear array that is representing the hash grid.
   cell_ = new Cell[ xyzCellCount_ ];

   // Initialization of each cell - i.e., initially setting the pointer to the body container to
   // NULL (=> no bodies are assigned to this hash grid yet!) and ...
   for( Cell* c  = cell_; c < cell_ + xyzCellCount_; ++c ) {
      c->bodies_ = NULL;
   }
   // ... setting up the neighborhood relationship (using the offset array).
   initializeNeighborOffsets();

   cellSpan_        = cellSpan;
   inverseCellSpan_ = static_cast<real>( 1 ) / cellSpan;

   occupiedCells_.reserve( occupiedCellsVectorSize );

   bodyCount_ = 0;
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the HashGrid class.
 */
template< typename C >  // Type of the configuration
HashGrids<C>::HashGrid::~HashGrid()
{
   clear();

   for( Cell* c  = cell_; c < cell_ + xyzCellCount_; ++c ) {
      if( c->neighborOffset_ != stdNeighborOffset_ ) delete[] c->neighborOffset_;
   }
   delete[] cell_;
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adds a body to this hash grid.
 *
 * \param body The body that is about to be added to this hash grid.
 * \return void
 *
 * This function is called every time a new rigid body is added to this grid. If adding the body
 * will cause the total number of bodies assigned to this grid to exceed the enlargement threshold,
 * the size of this hash grid is increased in order to maintain a fixed minimal grid density (=
 * ratio of cells to bodies). This function may also be called during the update phase.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::add( BodyID body )
{
   // If adding the body will cause the total number of bodies assigned to this grid to exceed the
   // enlargement threshold, the size of this hash grid must be increased.
   if( bodyCount_ == enlargementThreshold_ ) enlarge();

   // Calculate (and store) the hash value (= the body's cell association) and ...
   size_t h = hash( body );
   body->setHash( h );

   // ... insert the body into the corresponding cell.
   Cell* cell = cell_ + h;
   add( body, cell );

   ++bodyCount_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removes a body from this hash grid.
 *
 * \param body The body that is about to be removed from this hash grid.
 * \return void
 *
 * This function is called every time a rigid body is removed from this grid. It may also be called
 * during the update phase.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::remove( BodyID body )
{
   // The stored hash value (= the body's cell association) is used in order to directly access the
   // cell from which this body will be removed.
   Cell* cell = cell_ + body->getHash();
   remove( body, cell );

   --bodyCount_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Updates the cell association of a body that is assigned to this grid.
 *
 * \param body The body whose cell association is being updated.
 * \return void
 *
 * Checks if a given rigid body can remain stored in its currently assigned cell - and if not,
 * removes the body from this cell and reassigns it to another cell (which is identified by
 * re-evaluating the body's hash value).
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::update( BodyID body )
{
   // The hash value is recomputed based on the body's current spatial location.
   size_t newHash = hash( body );
   size_t oldHash = body->getHash();

   // If this new hash value is identical to the hash value of the previous time step, the body
   // remains assigned to its current grid cell.
   if( newHash == oldHash ) return;

   // Only if the hash value changes, the cell association has to be changed, too - meaning, the
   // body has to be removed from its currently assigned cell and ...
   Cell* cell = cell_ + oldHash;
   remove( body, cell );

   body->setHash( newHash );

   // ... stored in the cell that corresponds to the new hash value.
   cell = cell_ + newHash;
   add( body, cell );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Processes this hash grid for colliding bodies.
 *
 * \param gridBodies Linear array of (handles to) all the bodies stored in this hash grid.
 * \param contacts Contact container for the generated contacts.
 * \return The number of bodies stored in this grid - which is the number of bodies stored in \a gridBodies.
 *
 * This function generates all contacts between all rigid bodies that are assigned to this grid. The
 * contacts are added to the contact container \a contacts. Moreover, a linear array that contains
 * (handles to) all bodies that are stored in this grid is returned in order to being able to check
 * these bodies against other bodies that are stored in grids with larger sized cells.
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
size_t HashGrids<C>::HashGrid::process( BodyID** gridBodies, Contacts& contacts ) const
{
   BodyID* bodies = new BodyID[ bodyCount_ ];
   *gridBodies    = bodies;

   // Iterate through all cells that are occupied by bodies (=> 'occupiedCells_') and ...
   for( typename CellVector::const_iterator cell = occupiedCells_.begin(); cell < occupiedCells_.end(); ++cell )
   {
      BodyVector* cellBodies = (*cell)->bodies_;

      // ... perform pairwise collision checks within each of these cells.
      for( typename BodyVector::iterator a = cellBodies->begin(); a < cellBodies->end(); ++a ) {
         for( typename BodyVector::iterator b = a + 1; b < cellBodies->end(); ++b ) {
            HashGrids<C>::collide( *a, *b, contacts );
         }
         *(bodies++) = *a;
      }

      // Moreover, check all the bodies that are stored in the currently processed cell against all
      // bodies that are stored in the first half of all directly adjacent cells.
      for( unsigned int i = 0; i < 13; ++i )
      {
         Cell*       nbCell   = (*cell) + (*cell)->neighborOffset_[i];
         BodyVector* nbBodies = nbCell->bodies_;

         if( nbBodies != NULL )
         {
            for( typename BodyVector::iterator a = cellBodies->begin(); a < cellBodies->end(); ++a ) {
               for( typename BodyVector::iterator b = nbBodies->begin(); b < nbBodies->end(); ++b ) {
                  HashGrids<C>::collide( *a, *b, contacts );
               }
            }
         }
      }
   }

   return bodyCount_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks all the bodies that are stored in \a bodies for collisions with the bodies that are
 *        stored in this grid.
 *
 * \param bodies Linear array of (handles to) all the bodies that are about to be checked for
 *        collisions with the bodies that are stored in this grid.
 * \param bodyCount The number of bodies that are stored in \a bodies.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * This function generates all contacts between the rigid bodies that are stored in \a bodies and
 * all the rigid bodies that are assigned to this grid. The contacts are added to the contact
 * container \a contacts.
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
void HashGrids<C>::HashGrid::processBodies( BodyID* bodies, size_t bodyCount, Contacts& contacts ) const
{
   // For each body 'a' that is stored in 'bodies' ...
   for( BodyID* a = bodies; a < bodies + bodyCount; ++a )
   {
      // ... calculate the body's cell association (=> "hash()") within this hash grid and ...
      Cell* cell = cell_ + hash( *a );

      // ... check 'a' against every body that is stored in this or in any of the directly adjacent
      // cells. Note: one entry in the offset array of a cell is always referring back to the cell
      // itself. As a consequence, a specific cell X and all of its neighbors can be addressed by
      // simply iterating through all entries of X's offset array!
      for( unsigned int i = 0; i < 27; ++i )
      {
         Cell*       nbCell   = cell + cell->neighborOffset_[i];
         BodyVector* nbBodies = nbCell->bodies_;

         if( nbBodies != NULL ) {
            for( typename BodyVector::iterator b = nbBodies->begin(); b < nbBodies->end(); ++b ) {
               HashGrids<C>::collide( *a, *b, contacts );
            }
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clears the hash grid.
 *
 * \return void
 *
 * This function removes all (the handles to) the bodies that are assigned to this hash grid from
 * the body containers of the grid's cells.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::clear()
{
   for( typename CellVector::iterator cell = occupiedCells_.begin(); cell < occupiedCells_.end(); ++cell ) {
      delete (*cell)->bodies_;
      (*cell)->bodies_ = NULL;
   }
   occupiedCells_.clear();
   bodyCount_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets up the neighborhood relationships for all grid cells.
 *
 * \return void
 *
 * This function is used to initialize the offset arrays of all grid cells. The offsets are required
 * for ensuring fast direct access to all directly adjacent cells for each cell in the hash grid.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::initializeNeighborOffsets()
{
   offset_t xc   = static_cast<offset_t>( xCellCount_ );
   offset_t yc   = static_cast<offset_t>( yCellCount_ );
   offset_t zc   = static_cast<offset_t>( zCellCount_ );
   offset_t xyc  = static_cast<offset_t>( xyCellCount_ );
   offset_t xyzc = static_cast<offset_t>( xyzCellCount_ );

   // Initialization of the grid-global offset array that is valid for all inner cells in the hash grid.
   unsigned int i = 0;
   for( offset_t zz = -xyc; zz <= xyc; zz += xyc ) {
      for( offset_t yy = -xc; yy <= xc; yy += xc ) {
         for( offset_t xx = -1; xx <= 1; ++xx, ++i ) {
            stdNeighborOffset_[i] = xx + yy + zz;
         }
      }
   }

   // Allocation and initialization of the offset arrays of all the border cells. All inner cells
   // are set to point to the grid-global offset array.
   Cell* c = cell_;
   for( offset_t z = 0; z < zc; ++z ) {
      for( offset_t y = 0; y < yc; ++y ) {
         for( offset_t x = 0; x < xc; ++x, ++c ) {

            /* border cell */
            if( x == 0 || x == (xc - 1) || y == 0 || y == (yc - 1) || z == 0 || z == (zc - 1) ) {

               c->neighborOffset_ = new offset_t[27];

               i = 0;
               for( offset_t zz = -xyc; zz <= xyc; zz += xyc )
               {
                  offset_t zo = zz;
                  if( z == 0 && zz == -xyc ) {
                     zo = xyzc - xyc;
                  }
                  else if( z == (zc - 1) && zz == xyc ) {
                     zo = xyc - xyzc;
                  }

                  for( offset_t yy = -xc; yy <= xc; yy += xc )
                  {
                     offset_t yo = yy;
                     if( y == 0 && yy == -xc ) {
                        yo = xyc - xc;
                     }
                     else if( y == (yc - 1) && yy == xc ) {
                        yo = xc - xyc;
                     }

                     for( offset_t xx = -1; xx <= 1; ++xx, ++i ) {

                        offset_t xo = xx;
                        if( x == 0 && xx == -1 ) {
                           xo = xc - 1;
                        }
                        else if( x == (xc - 1) && xx == 1 ) {
                           xo = 1 - xc;
                        }

                        c->neighborOffset_[i] = xo + yo + zo;
                     }
                  }
               }
            }
            /* inner cell */
            else {
               c->neighborOffset_ = stdNeighborOffset_;
            }
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Computes the hash value (= cell association) of a given rigid body.
 *
 * \param body The body whose hash value is about to be calculated.
 * \return The hash value (=cell association) of the body.
 *
 * The hash calculation uses modulo operations in order to spatially map entire blocks of connected
 * cells to the origin of the coordinate system. This block of cells at the origin of the coordinate
 * system that is filled with all the bodies of the simulation is referred to as the hash grid. The
 * key feature, and ultimately the advantage, of hash grids is the fact that two adjacent cells that
 * are located anywhere in the simulation space are mapped to two cells that are still adjacent in
 * the hashed storage structure.
 *
 * Note that the modulo calculations are replaced with fast bitwise AND operations - hence, the
 * spatial dimensions of the hash grid must be restricted to powers of two!
 */
template< typename C >  // Type of the configuration
size_t HashGrids<C>::HashGrid::hash( BodyID body ) const
{
   real x = body->getAABB()[0];
   real y = body->getAABB()[1];
   real z = body->getAABB()[2];

   size_t xHash;
   size_t yHash;
   size_t zHash;

   if( x < 0 ) {
      real i = ( -x ) * inverseCellSpan_;
      xHash  = xCellCount_ - 1 - ( static_cast<size_t>( i ) & xHashMask_ );
   }
   else {
      real i = x * inverseCellSpan_;
      xHash  = static_cast<size_t>( i ) & xHashMask_;
   }

   if( y < 0 ) {
      real i = ( -y ) * inverseCellSpan_;
      yHash  = yCellCount_ - 1 - ( static_cast<size_t>( i ) & yHashMask_ );
   }
   else {
      real i = y * inverseCellSpan_;
      yHash  = static_cast<size_t>( i ) & yHashMask_;
   }

   if( z < 0 ) {
      real i = ( -z ) * inverseCellSpan_;
      zHash  = zCellCount_ - 1 - ( static_cast<size_t>( i ) & zHashMask_ );
   }
   else {
      real i = z * inverseCellSpan_;
      zHash  = static_cast<size_t>( i ) & zHashMask_;
   }

   return xHash + yHash * xCellCount_ + zHash * xyCellCount_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a body to a specific cell in this hash grid.
 *
 * \param body The body that is about to be added to this hash grid.
 * \param cell The cell the body is assigend to.
 * \return void
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::add( BodyID body, Cell* cell )
{
   using namespace pe::detection::coarse::hhg;

   // If this cell is already occupied by other bodies, which means the pointer to the body
   // container holds a valid address and thus the container itself is properly initialized, then
   // the body is simply added to this already existing body container. Note that the index position
   // is memorized (=> "body->setCellId()") in order to ensure constant time removal.
   if( cell->bodies_ != NULL ) {
      body->setCellId( cell->bodies_->size() );
      cell->bodies_->push_back( body );
   }

   // If, however, the cell is still empty, then the object container, first of all, must be created
   // (i.e., allocated) and properly initialized (i.e., sufficient initial storage capacity must be
   // reserved). Furthermore, the cell must be inserted into the grid-global vector 'occupiedCells_'
   // in which all cells that are currently occupied by bodies are recorded.
   else {
      cell->bodies_ = new BodyVector;
      cell->bodies_->reserve( cellVectorSize );

      body->setCellId( 0 );
      cell->bodies_->push_back( body );

      cell->occupiedCellsId_ = occupiedCells_.size();
      occupiedCells_.push_back( cell );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removes a body from a specific cell in this hash grid.
 *
 * \param body The body that is about to be removed from this hash grid.
 * \param cell The cell the body is removed from.
 * \return void
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::remove( BodyID body, Cell* cell )
{
   // If the body is the last body that is stored in this cell ...
   if( cell->bodies_->size() == 1 ) {

      // ... the cell's body container is destroyed and ...
      delete cell->bodies_;
      cell->bodies_ = NULL;

      // ... the cell is removed from the grid-global vector 'occupiedCells_' that records all
      // body-occupied cells. Since the cell memorized its index (=> 'occupiedCellsId_') in this
      // vector, it can be removed in constant time, O(1).
      if( cell->occupiedCellsId_ == occupiedCells_.size() - 1 ) {
         occupiedCells_.pop_back();
      }
      else {
         Cell* lastCell = occupiedCells_.back();
         occupiedCells_.pop_back();
         lastCell->occupiedCellsId_ = cell->occupiedCellsId_;
         occupiedCells_[ cell->occupiedCellsId_ ] = lastCell;
      }
   }
   // If the body is *not* the last body that is stored in this cell ...
   else
   {
      size_t cellId = body->getCellId();

      // ... the body is removed from the cell's body container. Since the body memorized its
      // index (=> 'cellId') in this container, it can be removed in constant time, O(1).
      if( cellId == cell->bodies_->size() - 1 ) {
         cell->bodies_->pop_back();
      }
      else {
         BodyID lastElement = cell->bodies_->back();
         cell->bodies_->pop_back();
         lastElement->setCellId( cellId );
         (*cell->bodies_)[ cellId ] = lastElement;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the number of cells used by this hash grid.
 *
 * \return void
 *
 * In order to handle an initially unknown and ultimately arbitrary number of bodies, the hash grid,
 * starting with a rather small number of cells at the time of its creation, must have the ability
 * to grow as new bodies are inserted. Therefore, if by inserting a body into this hash grid the
 * associated grid density - that is the ratio of cells to bodies - drops below the threshold
 * specified by \a minimalGridDensity, or in other words, if the number of bodies grows larger than
 * specified by \a enlargementThreshold_, the number of cells in each coordinate direction is
 * doubled (thus the total number of grid cells is increased by a factor of 8).
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::HashGrid::enlarge()
{
   using namespace pe::detection::coarse::hhg;

   BodyID* bodies = new BodyID[ bodyCount_ ];
   BodyID* body   = bodies;

   // All bodies that are assigned to this grid are temporarily removed, ...
   for( typename CellVector::iterator cell = occupiedCells_.begin(); cell < occupiedCells_.end(); ++cell ) {
      BodyVector* cellBodies = (*cell)->bodies_;
      for( typename BodyVector::iterator e = cellBodies->begin(); e < cellBodies->end(); ++e ) {
         *(body++) = *e;
      }
   }

   // ... the grid's current data structures are deleted, ...
   clear();

   for( Cell* c  = cell_; c < cell_ + xyzCellCount_; ++c ) {
      if( c->neighborOffset_ != stdNeighborOffset_ ) delete[] c->neighborOffset_;
   }
   delete[] cell_;

   // ... the number of cells is doubled in each coordinate direction, ...
   xCellCount_  *= 2;
   yCellCount_  *= 2;
   zCellCount_  *= 2;

   xHashMask_    = xCellCount_ - 1;
   yHashMask_    = yCellCount_ - 1;
   zHashMask_    = zCellCount_ - 1;

   xyCellCount_  = xCellCount_ * yCellCount_;
   xyzCellCount_ = xyCellCount_ * zCellCount_;

   // ... a new threshold for enlarging this hash grid is set, ...
   enlargementThreshold_ = xyzCellCount_ / minimalGridDensity;

   // ... a new linear array of cells representing this enlarged hash grid is allocated and ...
   cell_ = new Cell[ xyzCellCount_ ];

   // ... initialized, and finally ...
   for( Cell* c  = cell_; c < cell_ + xyzCellCount_; ++c ) {
      c->bodies_ = NULL;
   }
   initializeNeighborOffsets();

   // ... all previously removed bodies are reinserted.
   for( BodyID* p = bodies; p < body; ++p ) {   
      add(*p);
   }
   delete[] bodies;
}
//*************************************************************************************************








//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the HashGrids class.
 *
 * \param bodystorage Reference to the general body storage.
 *
 * Note that all (local, global and remote) must be contained in the body storage.
 */
template< typename C >  // Type of the configuration
HashGrids<C>::HashGrids( BS& bodystorage )
   : bodystorage_( bodystorage )
   , bodystorageShadowCopies_( bodystorage )
{
   using namespace pe::detection::coarse::hhg;

   nonGridBodies_.reserve( gridActivationThreshold );

   // As long as the number of bodies is less-or-equal than specified by 'gridActivationThreshold',
   // no hierarchy of hash grids is constructed. Instead, all bodies are simply stored in
   // 'nonGridBodies_' and pairwise collision checks are performed.
   gridActive_ = false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the HashGrids class.
 *
 * \param bodystorage Reference to the central body storage.
 * \param bodystorageShadowCopies Reference to the body storage containing all body shadow copies.
 */
template< typename C >  // Type of the configuration
HashGrids<C>::HashGrids( BS& bodystorage, BS& bodystorageShadowCopies )
   : bodystorage_( bodystorage )
   , bodystorageShadowCopies_( bodystorageShadowCopies )
{
   using namespace pe::detection::coarse::hhg;

   nonGridBodies_.reserve( gridActivationThreshold );

   // As long as the number of bodies is less-or-equal than specified by 'gridActivationThreshold',
   // no hierarchy of hash grids is constructed. Instead, all bodies are simply stored in
   // 'nonGridBodies_' and pairwise collision checks are performed.
   gridActive_ = false;
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the HashGrids class.
 */
template< typename C >  // Type of the configuration
HashGrids<C>::~HashGrids()
{
   // Delete all grids that are stored in the grid hierarchy (=> gridList_).
   for( typename GridList::iterator grid = gridList_.begin(); grid != gridList_.end(); ++grid ) {
      delete (*grid);
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a rigid body with the coarse collision detector.
 *
 * \param body The body to be registered with the coarse collision detector.
 * \return void
 *
 * This function is a requirement for all coarse collision detectors. It is automatically
 * called every time a new rigid body is added to the simulation world.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::add( BodyID body )
{
   // The body is marked as being added to 'bodiesToAdd_' by setting the grid pointer to NULL and
   // setting the cell-ID to '0'. Additionally, the hash value is used to memorize the body's
   // index position in the 'bodiesToAdd_' vector.
   body->setGrid  ( NULL );
   body->setHash  ( bodiesToAdd_.size() );
   body->setCellId( 0 );

   // Temporarily add the body to 'bodiesToAdd_'. As soon as "findContacts()" is called, all
   // bodies stored in 'bodiesToAdd_' are finally inserted into the data structure.
   bodiesToAdd_.push_back( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a rigid body from the coarse collision detector.
 *
 * \param body The body to be deregistered from the coarse collision detector.
 * \return void
 *
 * This function is a requirement for all coarse collision detectors. It is automatically
 * called every time a rigid body is removed from the simulation world.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::remove( BodyID body )
{
   HashGrid* grid = static_cast<HashGrid*>( body->getGrid() );

   // The body is stored in a hash grid from which it must be removed.
   if( grid != NULL ) {
      grid->remove( body );
   }
   // The body's grid pointer is equal to NULL.
   // => The body is either stored in 'bodiesToAdd_' (-> cell-ID = 0) or 'nonGridBodies_' (-> cell-ID = 1).
   else {
      if( body->getCellId() == 0 ) {
         // the body's hash value => index of this body in 'bodiesToAdd_'
         if( body->getHash() == bodiesToAdd_.size() - 1 ) {
            bodiesToAdd_.pop_back();
         }
         else {
            BodyID lastElement = bodiesToAdd_.back();
            bodiesToAdd_.pop_back();
            lastElement->setHash( body->getHash() );
            bodiesToAdd_[ body->getHash() ] = lastElement;
         }
      }
      else {
         // the body's hash value => index of this body in 'nonGridBodies_'
         if( body->getHash() == nonGridBodies_.size() - 1 ) {
            nonGridBodies_.pop_back();
         }
         else {
            BodyID lastElement = nonGridBodies_.back();
            nonGridBodies_.pop_back();
            lastElement->setHash( body->getHash() );
            nonGridBodies_[ body->getHash() ] = lastElement;
         }
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact generation between colliding rigid bodies.
 *
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * This function generates all contacts between all registered rigid bodies. The contacts are
 * added to the contact container \a contacts.
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
void HashGrids<C>::findContacts( Contacts& contacts )
{
   using namespace pe::detection::coarse::hhg;
   typedef typename BS::Iterator Iterator;

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Finding the contacts via the hierarchical hash grids algorithm...";
   }

   // ----- UPDATE PHASE ----- //

   // Finally add all bodies that were temporarily stored in 'bodiesToAdd_' to the data structure.
   if( bodiesToAdd_.size() > 0 )
   {
      for( typename BodyVector::iterator body = bodiesToAdd_.begin(); body < bodiesToAdd_.end(); ++body ) {

         if( gridActive_ ) addGrid( *body );
         else              addList( *body );
      }
      bodiesToAdd_.clear();
   }

   // Update the data structure (=> adapt to the current body distribution) by taking care of
   // moved, rotated and/or deformed bodies.
   if( gridActive_ )
   {
      // 1) Update strategy: All bodies are simultaneously removed from the data structure, and
      //                     immediately afterwards they are reinserted based on their current sizes
      //                     and spatial locations - which may or may not have changed since the
      //                     last time step.
      //
      // for( typename GridList::iterator grid = gridList_.begin(); grid != gridList_.end(); ++grid ) {
      //    (*grid)->clear();
      // }
      //
      // {
      //    const Iterator end( bodystorage_.end() );
      //    for( Iterator b = bodystorage_.begin(); b != end; ++b ) {
      //
      //       BodyID    body = *b;
      //       HashGrid* grid = static_cast<HashGrid*>( body->getGrid() );
      //
      //       if( grid != NULL ) {
      //
      //          real size     = body->getAABBSize();
      //          real cellSpan = grid->getCellSpan();
      //
      //          if( size >= cellSpan || size < ( cellSpan / hierarchyFactor ) ) {
      //             addGrid( body );
      //          }
      //          else {
      //             grid->add( body );
      //          }
      //       }
      //    }
      // }
      //
      // if( &bodystorage_ != &bodystorageShadowCopies_ ){
      //    const Iterator end( bodystorageShadowCopies_.end() );
      //    for( Iterator b = bodystorageShadowCopies_.begin(); b != end; ++b ) {
      //
      //       BodyID    body = *b;
      //       HashGrid* grid = static_cast<HashGrid*>( body->getGrid() );
      //
      //       if( grid != NULL ) {
      //
      //          real size     = body->getAABBSize();
      //          real cellSpan = grid->getCellSpan();
      //
      //          if( size >= cellSpan || size < ( cellSpan / hierarchyFactor ) ) {
      //             addGrid( body );
      //          }
      //          else {
      //             grid->add( body );
      //          }
      //       }
      //    }
      // }

      // 2) Update strategy: For every body, the grid association is recomputed based on the body's
      //                     current size (=> size of its AABB -> length of the longest edge of the
      //                     AABB, which most likely changes every time the body is rotated or
      //                     deformed):
      //
      //                      - If the new grid association is identical to the current grid
      //                        association, the body may remain assigned to its current grid, yet
      //                        still the body's cell association within this grid has to be checked
      //                        and potentially changed if the body was moved (=> "grid->update()").
      //
      //                      - If the grid association has changed, the body must be removed from
      //                        its current grid (=> "grid->remove()") and reassigned to a grid with
      //                        suitably sized cells (=> "addGrid()").

      {
         const Iterator end( bodystorage_.end() );
         for( Iterator b = bodystorage_.begin(); b != end; ++b )
         {
            BodyID body = *b;
            HashGrid* grid = static_cast<HashGrid*>( body->getGrid() );

            if( grid != NULL )
            {
               real size     = body->getAABBSize();
               real cellSpan = grid->getCellSpan();

               if( size >= cellSpan || size < ( cellSpan / hierarchyFactor ) ) {
                  grid->remove( body );
                  addGrid( body );
               }
               else {
                  grid->update( body );
               }
            }
         }
      }

      if( &bodystorage_ != &bodystorageShadowCopies_ ) {
         const Iterator end( bodystorageShadowCopies_.end() );
         for( Iterator b = bodystorageShadowCopies_.begin(); b != end; ++b )
         {
            BodyID body = *b;
            HashGrid* grid = static_cast<HashGrid*>( body->getGrid() );

            if( grid != NULL )
            {
               real size     = body->getAABBSize();
               real cellSpan = grid->getCellSpan();

               if( size >= cellSpan || size < ( cellSpan / hierarchyFactor ) ) {
                  grid->remove( body );
                  addGrid( body );
               }
               else {
                  grid->update( body );
               }
            }
         }
      }
   }

   // ----- DETECTION STEP ----- //

   // Contact generation by traversing through all hash grids (which are sorted in ascending order
   // with respect to the size of their cells).
   for( typename GridList::iterator grid = gridList_.begin(); grid != gridList_.end(); ++grid ) {

      // Contact generation for all bodies stored in the currently processed grid 'grid'.
      BodyID* bodies     = NULL;
      size_t  bodyCount = (*grid)->process( &bodies, contacts );

      if( bodyCount > 0 ) {

         // Test all bodies stored in 'grid' against bodies stored in grids with larger sized cells.
         typename GridList::iterator nextGrid = grid;
         for( ++nextGrid; nextGrid != gridList_.end(); ++nextGrid ) {
            (*nextGrid)->processBodies( bodies, bodyCount, contacts );
         }

         // Test all bodies stored in 'grid' against all bodies stored in 'nonGridBodies_'.
         BodyID* bodiesEnd = bodies + bodyCount;
         for( BodyID* a = bodies; a < bodiesEnd; ++a ) {
            for( typename BodyVector::iterator b = nonGridBodies_.begin(); b < nonGridBodies_.end(); ++b ) {
               collide( *a, *b, contacts );
            }
         }
      }

      delete[] bodies;
   }

   // Pairwise test (=> contact generation) for all bodies that are stored in 'nonGridBodies_'.
   for( typename BodyVector::iterator a = nonGridBodies_.begin(); a < nonGridBodies_.end(); ++a ) {
      for( typename BodyVector::iterator b = a + 1; b < nonGridBodies_.end(); ++b ) {
         collide( *a, *b, contacts );
      }
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( contacts.isEmpty() )
         log << "      no contacts found!\n";
      else {
         log << "      State of the contacts:\n";
         for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c ) {
            log << "         Contact " << c->getID() << " (" << c->getType() << "):"
                << " pos = " << c->getPosition()
                << " , n = " << c->getNormal()
                << " , dist = " << c->getDistance()
                << " , vrel = " << c->getNormalRelVel()
                << " , cor = " << c->getRestitution()
                << "\n";
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clears the coarse collision detector.
 *
 * \return void
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::clear()
{
   for( typename GridList::iterator grid = gridList_.begin(); grid != gridList_.end(); ++grid ) {
      delete (*grid);
   }
   gridList_.clear();

   gridActive_ = false;

   nonGridBodies_.clear();

   bodiesToAdd_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks whether two bodies are colliding or not, and if so generates the related points
 *        of contact.
 *
 * \param a The first body.
 * \param b The second body.
 * \param contacts Contact container for the generated contacts.
 * \return void
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
void HashGrids<C>::collide( BodyID a, BodyID b, Contacts& contacts )
{
   if( ( !a->isFixed() || !b->isFixed() ) &&        // Ignoring contacts between two fixed bodies
       ( a->getAABB().overlaps( b->getAABB() ) ) )  // Testing for overlapping bounding boxes
   {
      FineDetector::collide( a, b, contacts );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adds a body to the hierarchical hash grids data structure.
 *
 * \param body The body that is about to be added to the hierarchical hash grids data structure.
 * \return void
 *
 * If the usage of hierarchical hash grids is activated, this function is called every time a new
 * rigid body is added to the data structure. It may also be called during the update phase.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::addGrid( BodyID body )
{
   using namespace pe::detection::coarse::hhg;

   real size = ( body->isFinite() ) ? body->getAABBSize() : static_cast<real>( -1 );

   // If the body is finite in size, it must be assigned to a grid with suitably sized cells.
   if( size > 0 )
   {
      HashGrid* grid = NULL;

      if( gridList_.empty() )
      {
         // If no hash grid yet exists in the hierarchy, an initial hash grid is created
         // based on the body's size.

         grid = new HashGrid( size * std::sqrt( hierarchyFactor ) );
      }
      else
      {
         // Check the hierarchy for a hash grid with suitably sized cells - if such a grid does not
         // yet exist, it will be created.

         real cellSpan = 0;
         for( typename GridList::iterator g = gridList_.begin(); g != gridList_.end(); ++g )
         {
            grid     = *g;
            cellSpan = grid->getCellSpan();

            if( size < cellSpan )
            {
               cellSpan /= hierarchyFactor;
               if( size < cellSpan ) {
                  while( size < cellSpan ) cellSpan /= hierarchyFactor;
                  grid = new HashGrid( cellSpan * hierarchyFactor );
                  gridList_.insert( g, grid );
               }

               grid->add( body );
               body->setGrid( static_cast<void*>( grid ) );

               return;
            }
         }

         while( size >= cellSpan) cellSpan *= hierarchyFactor;
         grid = new HashGrid( cellSpan );
      }

      grid->add( body );
      body->setGrid( static_cast<void*>( grid ) );

      gridList_.push_back( grid );

      return;
   }

   // The body - which is infinite in size - is marked as being added to 'nonGridBodies_' by setting
   // the grid pointer to NULL and setting the cell-ID to '1'. Additionally, the hash value is used
   // to memorize the body's index position in the 'nonGridBodies_' vector.

   body->setGrid  ( NULL );
   body->setHash  ( nonGridBodies_.size() );
   body->setCellId( 1 );

   nonGridBodies_.push_back( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a body to the data structure (the usage of hash grids is not yet activated!).
 *
 * \param body The body that is about to be added to the data structure.
 * \return void
 *
 * As long as the number of bodies is less-or-equal than specified by \a gridActivationThreshold,
 * this function is called every time a new rigid body is added to the data structure. The body then
 * is stored in \a nonGridBodies_ - so that later during the detection step pairwise collision
 * checks for all bodies stored in \a nonGridBodies_ can be performed.
 *
 * However, the moment the threshold is exceeded, all bodies are removed from the array and finally
 * added to the grid hierarchy - thereby creating the initial set of hash grids which are adapted to
 * the just inserted bodies. Once this has happened, the usage of the hierarchical hash grids is
 * activated irrevocably, which means the coarse collision detection phase will continue to use the
 * hierarchical hash grids, even if removing bodies from the simulation might cause the total number
 * of bodies to again drop below the threshold.
 */
template< typename C >  // Type of the configuration
void HashGrids<C>::addList( BodyID body )
{
   using namespace pe::detection::coarse::hhg;

   // If the threshold is exceeded ...
   if( nonGridBodies_.size() == gridActivationThreshold )
   {
      if( gridActivationThreshold > 0 )
      {
         BodyID* bodies = new BodyID[ gridActivationThreshold ];

         // ... all bodies stored in 'nonGridBodies_' are temporarily copied ...
         for( size_t i = 0; i < gridActivationThreshold; ++i ) {
            bodies[i] = nonGridBodies_[i];
         }

         // ... the 'nonGridBodies_' vector is cleared ...
         nonGridBodies_.clear();

         // ... the bodies are reinserted into the data structure - yet this time they are
         // inserted into the grid hierarchy (=> "addGrid()") ...
         for( size_t i = 0; i < gridActivationThreshold; ++i ) {
            addGrid( bodies[i] );
         }

         delete[] bodies;
      }

      addGrid( body );

      // ... and the usage of the hierarchical hash grids is activated irrevocably.
      gridActive_ = true;

      return;
   }

   // The body is marked as being added to 'nonGridBodies_' by setting the grid pointer to NULL and
   // setting the cell-ID to '1'. Additionally, the hash value is used to memorize the body's index
   // position in the 'nonGridBodies_' vector.

   body->setGrid  ( NULL );
   body->setHash  ( nonGridBodies_.size() );
   body->setCellId( 1 );

   nonGridBodies_.push_back( body );   
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks whether a certain number is equal to a power of two.
 *
 * \param number The number that is about to be checked.
 * \return \a true if the number is equal to a power of two, \a false if not.
 * 
 * This function is used to ensure that the number of cells in each coordinate dimension of a hash
 * grid is equal to a power of two so that the modulo calculations required for the hash value
 * computation can be replaced by bitwise AND operations.
 */
template< typename C >  // Type of the configuration
bool HashGrids<C>::powerOfTwo( size_t number )
{
   return ( ( number > 0 ) && ( ( number & ( number - 1 ) ) == 0 ) );
}
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
