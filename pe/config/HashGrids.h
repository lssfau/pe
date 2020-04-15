//=================================================================================================
/*!
 *  \file pe/config/HashGrids.h
 *  \brief Configuration of the hierarchical hash grids coarse collision detection algorithm
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


//*************************************************************************************************
/*!\brief The initial number of cells in x-direction of a newly created hash grid.
 * \ingroup config
 *
 * This value represents the initial number of cells of a newly created hash grid in x-direction.
 * The larger the value (i.e. the greater the number of cells of every newly created hash grid),
 * the more memory is required for the storage of the hash grid. Since the size of a hash grid is
 * increased at runtime in order to adapt to the number of currently inserted bodies, 16x16x16
 * is a suitable choice for the initial size of a newly created hash grid - it already consists
 * of four thousand cells, yet only requires a few hundred kilobytes of memory. Note that the
 * initial number of cells must both be greater-or-equal to 4 and equal to a power of two. Also
 * note that the initial number of cells does not necessarily have to be equal for all three
 * coordinate directions.
 */
const size_t xCellCount = 16;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The initial number of cells in y-direction of a newly created hash grid.
 * \ingroup config
 *
 * This value represents the initial number of cells of a newly created hash grid in y-direction.
 * The larger the value (i.e. the greater the number of cells of every newly created hash grid),
 * the more memory is required for the storage of the hash grid. Since the size of a hash grid is
 * increased at runtime in order to adapt to the number of currently inserted bodies, 16x16x16
 * is a suitable choice for the initial size of a newly created hash grid - it already consists
 * of four thousand cells, yet only requires a few hundred kilobytes of memory. Note that the
 * initial number of cells must both be greater-or-equal to 4 and equal to a power of two. Also
 * note that the initial number of cells does not necessarily have to be equal for all three
 * coordinate directions.
 */
const size_t yCellCount = 16;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The initial number of cells in z-direction of a newly created hash grid.
 * \ingroup config
 *
 * This value represents the initial number of cells of a newly created hash grid in z-direction.
 * The larger the value (i.e. the greater the number of cells of every newly created hash grid),
 * the more memory is required for the storage of the hash grid. Since the size of a hash grid is
 * increased at runtime in order to adapt to the number of currently inserted bodies, 16x16x16
 * is a suitable choice for the initial size of a newly created hash grid - it already consists
 * of four thousand cells, yet only requires a few hundred kilobytes of memory. Note that the
 * initial number of cells must both be greater-or-equal to 4 and equal to a power of two. Also
 * note that the initial number of cells does not necessarily have to be equal for all three
 * coordinate directions.
 */
const size_t zCellCount = 16;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The initial storage capaciy of a newly created grid cell body container.
 * \ingroup config
 *
 * This value specifies the initial storage capacity reserved for every grid cell body container,
 * i.e., the number of bodies that can initially be assigned to a grid cell with the need to
 * increase the storage capacity. The smaller this number, the more likely the storage capacity
 * of a body container must be increased, leading to potentially costly reallocation operations,
 * which generally involve the entire storage space to be copied to a new location. The greater
 * this number, the more memory is required. Rule of thumb:
 *
 *                        \f$ cellVectorSize = 2 \cdot hierarchyFactor^3 \f$
 */
const size_t cellVectorSize = 16;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The initial storage capacity of the grid-global vector.
 * \ingroup config
 *
 * This value specifies the initial storage capacity of the grid-global vector that keeps track
 * of all body-occupied cells. As long as at least one body is assigned to a certain cell, this
 * cell is recored in a grid-global list that keeps track of all body-occupied cells in order to
 * avoid iterating through all grid cells whenever all bodies that are stored in the grid need
 * to be addressed.
 */
const size_t occupiedCellsVectorSize = 256;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The minimal ratio of cells to bodies that must be maintained at any time.
 * \ingroup config
 *
 * This \a minimalGridDensity specifies the minimal ratio of cells to bodies that is allowed
 * before a grid grows.\n
 * In order to handle an initially unknown and ultimately arbitrary number of bodies, each hash
 * grid, starting with a rather small number of cells at the time of its creation, must have the
 * ability to grow as new bodies are inserted. Therefore, if by inserting a body into a hash grid
 * the associated grid density - that is the ratio of cells to bodies - drops below the threshold
 * specified by \a minimalGridDensity, the number of cells in each coordinate direction is doubled
 * (thus the total number of grid cells is increased by a factor of 8).
 *
 * Possible settings: any integral value greater than 0.
 */
const size_t minimalGridDensity = 8;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Activation threshold for the hierarchical hash grids coarse collision detection algorithm.
 * \ingroup config
 *
 * If the simulation only consists of a very small number of bodies, simply checking each body
 * against each other body proves to be faster than involving the far more complex mechanisms
 * of the hierarchical hash grids. In other words, despite its quadratic complexity, as long as
 * there are only a couple of bodies present in the simulation, the naive approach of conducting
 * pairwise checks for all existing bodies will always result in the best runtime performance. As
 * a consequence, a threshold is introduced, and as long as the number of bodies is less-or-equal
 * than specified by this threshold value, no hierarchy of hash grids is constructed and thus no
 * detection algorithm based on grids is applied.
 *
 * Possible settings: any integral value greater-or-equal to 0.
 */
const size_t gridActivationThreshold = 32;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The constant factor by which the cell size of any two successive grids differs.
 * \ingroup config
 *
 * This factor specifies the size difference of two successive grid levels of the hierarchical
 * hash grids. The grid hierarchy is constructed such that the cell size of any two successive
 * grids differs by a constant factor - the hierarchy factor \a hierarchyFactor. As a result,
 * the cell size \f$ c_k \f$ of grid \f$ k \f$ can be expressed as:
 *
 *                          \f$ c_k = c_0 \cdot hierarchyFactor^k \f$.
 *
 * Note that the hierarchy does not have to be dense, which means, if not every valid cell size
 * that can be generated is required, some in-between grids are not created. Consequently, the
 * cell size of two successive grids differs by a factor of \f$ hierarchyFactor^x \f$, with x
 * being an integral value that is not necessarily equal to 1.
 *
 * The larger the ratio between the cell size of two successive grids, the more bodies are
 * potentially assigned to one single cell, but overall fewer grids have to be used. On the other
 * hand, the smaller the ratio between the cell size of two successive grids, the fewer bodies
 * are assigned to one single cell, but overall more grids have to be created. Hence, the number
 * of bodies that are stored in one single cell is inversely proportional to the number of grids
 * which are in use. Unfortunately, minimizing the number of bodies that are potentially assigned
 * to the same cell and at the same time also minimizing the number of grids in the hierarchy are
 * two opposing goals. In general - based on the evaluation of a number of different scenarios -
 * the best choice seems to be a hierarchy factor that is equal to 2.0.
 *
 * Possible settings: any floating point value that is greater than 1.0.
 */
const real hierarchyFactor = real(2);
//*************************************************************************************************
