//=================================================================================================
/*!
 *  \file pe/core/OpenCLBodyProperties.h
 *  \brief Header file for the OpenCLBodyProperties class
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

#ifndef _PE_CORE_OPENCLBODYPROPERTIES_H_
#define _PE_CORE_OPENCLBODYPROPERTIES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <gpusolve.hpp>
#include <pe/util/logging/DebugSection.h>
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
/*!\brief Body attributes on an OpenCL device.
 * \ingroup opencl
 *
 * The object allocates storage for body attributes on the active OpenCL device. The active device
 * must be selected before instantiating this class. The OpenCLBodyProperties class is intended
 * to be instantiated and driven by the OpenCLBodyManager. Typical attributes are mass, inertia
 * tensor, positions, velocities.
 */
class OpenCLBodyProperties {
private:
   //**Type definitions****************************************************************************
   typedef gpusolve::opencl::vec4<float>   float4;       //!< A quadruple of floats.
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline OpenCLBodyProperties( size_t n = 0 );
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
   inline void contactsHostAccess( bool active );
   inline void hostAccess( bool active );
   //@}
   //**********************************************************************************************


private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;                            //!< The number of bodies in the OpenCL body storage.
   size_t sizeAllocated_;                   //!< The number of bodies allocated in the OpenCL body storage.
   static const size_t growthFactor_ = 2;   //!< The growth factor of the allocated memory chunks in case of reallocation.
   //@}
   //**********************************************************************************************


public:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   gpusolve::memory::Vector<float4,   gpusolve::interface::opencl> gpos_;         //!< The global positions.
   gpusolve::memory::Vector<float4,   gpusolve::interface::opencl> v_;            //!< The linear velocities.
   gpusolve::memory::Vector<float4,   gpusolve::interface::opencl> w_;            //!< The angular velocities.
   gpusolve::memory::Vector<float,    gpusolve::interface::opencl> invMass_;      //!< The inverse masses.
   gpusolve::memory::Vector<float4,   gpusolve::interface::opencl> invI_;         //!< The inverse inertia tensors with three entries per body - one for each row of the inertia tensor.
   gpusolve::memory::Vector<uint32_t, gpusolve::interface::opencl> status_;       //!< Identifies fixed bodies by non-zero values.
   gpusolve::memory::Vector<uint32_t, gpusolve::interface::opencl> mutex_;        //!< Mutex storage for atomic velocity updates.
   gpusolve::memory::Vector<uint32_t, gpusolve::interface::opencl> contacts_;     //!< Marks which contact colors are next to each body.
   gpusolve::memory::Vector<float,    gpusolve::interface::opencl> maxVelocity_;  //!< The maximum linear velocity storage for reduction by the ReduceMaxVelocity kernel for debugging purposes.
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
/*!\brief Constructor for the OpenCLBodyProperties class.
 *
 * \param n The number of bodies.
 */
inline OpenCLBodyProperties::OpenCLBodyProperties( size_t n )
   : size_(0)
   , sizeAllocated_(0)
   , gpos_()
   , v_()
   , w_()
   ,invMass_()
   , invI_()
   , status_()
   , contacts_()
   , maxVelocity_()
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
/*!\brief Resizes the body attribute storage.
 *
 * \param n The number of bodies.
 * \return void
 *
 * Requires rebuilding of invI_, mutex_ and contacts_ data (on reallocation).
 */
inline void OpenCLBodyProperties::resize( size_t n ) {
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
      std::cerr << "resizing OpenCLBodyProperties from " << sizeOld << " to ";
      std::cerr << size_ << " (" << sizeAllocated_ << ")" << std::endl;
   }

   gpos_.resize     ( sizeAllocated_, true );
   v_.resize        ( sizeAllocated_, true );
   w_.resize        ( sizeAllocated_, true );
   invMass_.resize  ( sizeAllocated_, true );
   invI_.resize     ( 3 * sizeAllocated_, false );      // TODO: can't keep values, has to be rebuilt!
   status_.resize   ( sizeAllocated_, true );
   mutex_.resize    ( sizeAllocated_, false );
   contacts_.resize ( sizeAllocated_, false );

   // TODO: handle blocksize globally
   maxVelocity_.resize( gpusolve::util::math::divCeil<128>( sizeAllocated_ ) );
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Maps or unmaps the OpenCL device memory chunk of the contacts_ attribute to the host.
 *
 * \param active Activates or deactivates host access to device memory chunk of the contacts_
 *               attribute.
 * \return void
 */
inline void OpenCLBodyProperties::contactsHostAccess( bool active ) {
   if( active ) {
      contacts_.enableAccess<gpusolve::memoryaccess::host>();
   }
   else {
      contacts_.disableAccess<gpusolve::memoryaccess::host>();
   }
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Maps or unmaps the OpenCL device memory chunks to the host.
 *
 * \param active Activates or deactivates host access to device memory.
 * \return void
 *
 * Exceptions are mutex_, contacts_ and maxVelocity_.
 */
inline void OpenCLBodyProperties::hostAccess( bool active ) {
   if( active ) {
      gpos_.enableAccess<gpusolve::memoryaccess::host>();
      v_.enableAccess<gpusolve::memoryaccess::host>();
      w_.enableAccess<gpusolve::memoryaccess::host>();
      invMass_.enableAccess<gpusolve::memoryaccess::host>();
      invI_.enableAccess<gpusolve::memoryaccess::host>();
      status_.enableAccess<gpusolve::memoryaccess::host>();
   }
   else {
      gpos_.disableAccess<gpusolve::memoryaccess::host>();
      v_.disableAccess<gpusolve::memoryaccess::host>();
      w_.disableAccess<gpusolve::memoryaccess::host>();
      invMass_.disableAccess<gpusolve::memoryaccess::host>();
      invI_.disableAccess<gpusolve::memoryaccess::host>();
      status_.disableAccess<gpusolve::memoryaccess::host>();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current size of the body storage.
 *
 * \return The current number of bodies in the storage.
 */
inline size_t OpenCLBodyProperties::size() const {
   return size_;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the current allocated size of the body storage.
 *
 * \return The number of bodies fitting into the body storage without reallocation.
 */
inline size_t OpenCLBodyProperties::sizeAllocated() const {
   return sizeAllocated_;
}
//*************************************************************************************************

} // namespace pe

#endif
