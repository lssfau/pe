//=================================================================================================
/*!
 *  \file pe/core/OpenCLUpdateCache.h
 *  \brief Header file for the OpenCLUpdateCache class
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

#ifndef _PE_CORE_OPENCLUPDATECACHE_H_
#define _PE_CORE_OPENCLUPDATECACHE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <gpusolve.hpp>
#include <algorithm>
#include <iostream>
#include <pe/util/logging/DebugSection.h>
#include <set>
#include <vector>


namespace pe {


//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup opencl OpenCL
 * \ingroup core
 */
/*!\brief Cache for velocity updates reduction.
 * \ingroup opencl
 *
 * The object allocates storage on the active OpenCL device for velocity updates due to contact
 * reaction changes. The updates are grouped by bodies to enable a simple reduction of multiple
 * velocity updates to a total velocity update of each body. The active device must be selected
 * before instantiating this class. The OpenCLUpdateCache class is intended to be instantiated
 * by the OpenCLSolver and driven by the OpenCLPGSSolver. The layout of the update cache is
 * described by the reduction information in the OpenCLContacts class. Typically velocity updates
 * due to contact reaction changes of a single color are stored so that the size of the update
 * cache is geared to the color with the maximum number of contacts. The data is written to
 * in the CalcVelocities() kernel, reduced in the ReduceVelocities() kernel and transfered
 * to body centric data structures in the UpdateVelocities() kernel.
 */
class OpenCLUpdateCache {
   //**Type definitions****************************************************************************
   /*!\name Type definitions */
   //@{
   typedef gpusolve::opencl::vec4<float> float4;
   typedef gpusolve::memory::Vector<float4,   gpusolve::interface::opencl> Float4Vec;
   typedef gpusolve::memory::Vector<uint32_t, gpusolve::interface::opencl> UintVec;
   //@}
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline OpenCLUpdateCache();
   //@}
   //**********************************************************************************************
   
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t size() const;
   inline size_t sizeAllocated() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void resize( size_t n );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;                            //!< The number of velocity updates in the update cache.
   size_t sizeAllocated_;                   //!< The number of velocity updates allocated in the update cache.
   static const size_t growthFactor_ = 2;   //!< The growth factor of the allocated memory chunks in case of reallocation.
   //@}
   //**********************************************************************************************

public:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   UintVec count_;   /*!< Auxiliary reduction data. Numbers the elements of each segment to be
                          reduced consecutively. */
   Float4Vec v_;     /*!< The linear velocity changes caused by each contact reaction change in one
                          color sweep of the solver. */
   Float4Vec w_;     /*!< The angular velocity changes caused by each contact reaction change in
                          one color sweep of the solver. */
   //@}
   //**********************************************************************************************
};




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the OpenCLUpdateCache class.
 */
inline OpenCLUpdateCache::OpenCLUpdateCache()
   : size_(0)
   , sizeAllocated_(0)
   , count_()
   , v_()
   , w_()
{
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resizes the contact attribute storage.
 *
 * \param n The number of contacts.
 * \return void
 *
 * Requires rebuilding of all data on reallocation.
 */
void OpenCLUpdateCache::resize( size_t n ) {
   if( n == size_ )
      return;

   if( n <= sizeAllocated_ ) {
      size_ = n;
      return;
   }

   sizeAllocated_ = gpusolve::util::math::ceil<16>( std::max( n, sizeAllocated_ * growthFactor_ ) );
   size_t sizeOld = size_;
   size_ = n;

   pe_LOG_DEBUG_SECTION( log ) {
      std::cerr << "resizing OpenCLUpdateCache from " << sizeOld << " to ";
      std::cerr << size_ << " (" << sizeAllocated_ << ")" << std::endl;
   }

   count_.resize ( sizeAllocated_ );
   v_.resize     ( sizeAllocated_ );
   w_.resize     ( sizeAllocated_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current size of the update cache.
 *
 * \return The current number of velocity updates fitting into the update cache.
 */
inline size_t OpenCLUpdateCache::size() const {
   return size_;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the current allocated size of the update cache.
 *
 * \return The number of contacts fitting into the update cache without reallocation.
 */
inline size_t OpenCLUpdateCache::sizeAllocated() const {
   return sizeAllocated_;
}
//*************************************************************************************************

} // namespace pe

#endif
