//=================================================================================================
/*!
 *  \file pe/core/OpenCLContacts.h
 *  \brief Header file for the OpenCLContacts class
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

#ifndef _PE_CORE_OPENCLCONTACTS_H_
#define _PE_CORE_OPENCLCONTACTS_H_


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
/*!\brief Contact attributes on an OpenCL device.
 * \ingroup opencl
 *
 * The object allocates storage for contact attributes on the active OpenCL device. The active
 * device must be selected before instantiating this class. The OpenCLContacts class is intended
 * to be instantiated and driven by the OpenCLSolver. Typical attributes are contact normals,
 * positions and friction coefficients. Furthermore the contact storage also caches specific
 * body properties (inverse mass and inertia tensor, position and linear and angular velocity)
 * such that they can be accessed by contact indexing.
 */
class OpenCLContacts {
   //**Type definitions****************************************************************************
   /*!\name Type definitions */
   //@{
   typedef gpusolve::opencl::vec4<float>    float4;
   typedef gpusolve::opencl::vec4<uint32_t> uint4;
   typedef gpusolve::opencl::vec4<int32_t>  int4;
   typedef gpusolve::opencl::vec2<uint32_t> uint2;
   typedef gpusolve::opencl::vec2<int32_t>  int2;

   typedef gpusolve::memory::Vector<float4,    gpusolve::interface::opencl> Float4Vec;
   typedef gpusolve::memory::Vector<float,     gpusolve::interface::opencl> FloatVec;
   typedef gpusolve::memory::Vector<uint4,     gpusolve::interface::opencl> Uint4Vec;
   typedef gpusolve::memory::Vector<uint2,     gpusolve::interface::opencl> Uint2Vec;
   typedef gpusolve::memory::Vector<uint32_t,  gpusolve::interface::opencl> UintVec;
   typedef gpusolve::memory::Vector<int4,      gpusolve::interface::opencl> int4Vec;
   //@}
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline OpenCLContacts( size_t n = 0 );
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
   inline void hostAccess( bool active );
   //@}
   //**********************************************************************************************




private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;                            //!< The number of contacts in the OpenCL contact storage.
   size_t sizeAllocated_;                   //!< The number of contacts allocated in the OpenCL contact storage.
   static const size_t growthFactor_ = 2;   //!< The growth factor of the allocated memory chunks in case of reallocation.
   //@}
   //**********************************************************************************************


public:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::vector<uint32_t> offsets_;  /*!< Host array to map colors to array offsets of the first
                                         component. If an array has several components then these
                                         are spaced by the allocation size reported by
                                         sizeAllocated(). */

   UintVec status_;             //!< TODO document my purpose - 0: inactive contact, 1: active contact
   Float4Vec normal_;           /*!< Storage for the contact local coordinate frame. The three
                                     components are normal vector, first tangential and second
                                     tangential vector. */
   Float4Vec diagonalBlock_;    /*!< Stores the rows of the 3x3 diagonal blocks of the system
                                     matrix (three components). */
   Float4Vec phi_;              //!< Storage for the gap data (1 component).
   Float4Vec p_;                //!< Storage for the reaction impulses (1 component).
   Float4Vec r_;                /*!< Stores the contact position relative to the body position for
                                     each body involved in the contact (2 components). */
   FloatVec mu_;                //!< Stores the coefficient of friction (1 component).
   int4Vec reductionInfo_;      /*!< Maps contact indices to OpenCLUpdateCache indices (1
                                     component). The third and forth value of each quadruple are
                                     the indices into the OpenCLUpdateCache where the velocity
                                     updates due to a contact reaction change must be written to.
                                     There is one index for each of the two bodies per contact.
                                     The values one and two are either -1 for fixed bodies or a
                                     non-negative temporary value for calculating the other
                                     values. */
   Uint2Vec bodies_;            /*!< Stores the indices of the bodies involved in each contact (2
                                     components). */
   FloatVec residual_;          //!< Temporary storage for the residual reduction (1 component).
   FloatVec bodyInvMass_;       /*!< Cache for the inverse masses of the involved bodies (2
                                     components). */
   Float4Vec bodyInvInertia_;   /*!< Cache for the inverse inertia tensors of the involved bodies
                                     (3 components per body = 6 components). */
   Float4Vec bodyGPos_;         //!< Cache for the position of the involved bodies (2 components).
   Float4Vec bodyLinVel_;       /*!< Cache for the linear velocities of the involved bodies (2
                                     components). */
   Float4Vec bodyAngVel_;       /*!< Cache for the angular velocities of the involved bodies (2
                                     components). */
   //@}
   //**********************************************************************************************
};




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the OpenCLContacts class.
 *
 * \param n The number of contacts.
 */
inline OpenCLContacts::OpenCLContacts( size_t n )
   : size_(0)
   , sizeAllocated_(0)
   , offsets_()
   , status_()
   , normal_()
   , diagonalBlock_()
   , phi_()
   , p_()
   , r_()
   , reductionInfo_()
   , bodies_()
   , bodyInvMass_()
   , bodyInvInertia_()
   , bodyGPos_()
   , bodyLinVel_()
   , bodyAngVel_()
{
   if( n )
      resize( n );
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
void OpenCLContacts::resize( size_t n ) {
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
      std::cerr << "resizing OpenCLContacts from " << sizeOld << " to ";
      std::cerr << size_ << " (" << sizeAllocated_ << ")" << std::endl;
   }

   status_.resize         ( sizeAllocated_ );
   normal_.resize         ( 3 * sizeAllocated_ );
   diagonalBlock_.resize  ( 3 * sizeAllocated_ );
   phi_.resize            ( sizeAllocated_ );
   p_.resize              ( sizeAllocated_ );
   r_.resize              ( 2 * sizeAllocated_ );
   mu_.resize             ( 2 * sizeAllocated_ );
   reductionInfo_.resize  ( sizeAllocated_ );
   bodies_.resize         ( 2 * sizeAllocated_ );
   // TODO: handle blocksize globally
   residual_.resize       ( gpusolve::util::math::divCeil<128>( sizeAllocated_ ) * 32 );
   bodyInvMass_.resize    ( 2 * sizeAllocated_ );
   bodyInvInertia_.resize ( 6 * sizeAllocated_ );
   bodyGPos_.resize       ( 2 * sizeAllocated_ );
   bodyLinVel_.resize     ( 2 * sizeAllocated_ );
   bodyAngVel_.resize     ( 2 * sizeAllocated_ );
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Maps or unmaps the OpenCL device memory chunks to the host.
 *
 * \param active Activates or deactivates host access to device memory.
 * \return void
 *
 * Exceptions are residual_ and the body properties cache.
 */
void OpenCLContacts::hostAccess( bool active ) {
   if( active ) {
      status_.enableAccess<gpusolve::memoryaccess::host>();
      normal_.enableAccess<gpusolve::memoryaccess::host>();
      diagonalBlock_.enableAccess<gpusolve::memoryaccess::host>();
      phi_.enableAccess<gpusolve::memoryaccess::host>();
      p_.enableAccess<gpusolve::memoryaccess::host>();
      r_.enableAccess<gpusolve::memoryaccess::host>();
      mu_.enableAccess<gpusolve::memoryaccess::host>();
      reductionInfo_.enableAccess<gpusolve::memoryaccess::host>();
      bodies_.enableAccess<gpusolve::memoryaccess::host>();
   }
   else {
      status_.disableAccess<gpusolve::memoryaccess::host>();
      normal_.disableAccess<gpusolve::memoryaccess::host>();
      diagonalBlock_.disableAccess<gpusolve::memoryaccess::host>();
      phi_.disableAccess<gpusolve::memoryaccess::host>();
      p_.disableAccess<gpusolve::memoryaccess::host>();
      r_.disableAccess<gpusolve::memoryaccess::host>();
      mu_.disableAccess<gpusolve::memoryaccess::host>();
      reductionInfo_.disableAccess<gpusolve::memoryaccess::host>();
      bodies_.disableAccess<gpusolve::memoryaccess::host>();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current size of the contact storage.
 *
 * \return The current number of contacts in the storage.
 */
inline size_t OpenCLContacts::size() const {
   return size_;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the current allocated size of the contact storage.
 *
 * \return The number of contacts fitting into the contact storage without reallocation.
 */
inline size_t OpenCLContacts::sizeAllocated() const {
   return sizeAllocated_;
}
//*************************************************************************************************

} // namespace pe

#endif
