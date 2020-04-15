//=================================================================================================
/*!
 *  \file pe/core/OpenCLColoredContacts.h
 *  \brief Header file for the OpenCLColoredContacts class
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

#ifndef _PE_CORE_OPENCLCOLOREDCONTACTS_H_
#define _PE_CORE_OPENCLCOLOREDCONTACTS_H_


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
/*!\brief Auxiliary data structure for reordering contacts.
 * \ingroup opencl
 *
 * The object is used to manage information such as how many contacts of which color are
 * next to a body, how many reduction steps are necessary for a color etc. The active device must
 * be selected before instantiating this class. The OpenCLColoredContacts class is intended
 * to be instantiated and driven by the OpenCLSolver.
 */
class OpenCLColoredContacts {
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline OpenCLColoredContacts();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t involved( size_t body, size_t color ) const;
   inline size_t& involved( size_t body, size_t color );
   inline size_t offset( size_t body, size_t color ) const;
   inline size_t& offset( size_t body, size_t color );
   inline size_t base( size_t body, size_t color ) const;
   inline const gpusolve::memory::Vector<cl_uint, gpusolve::interface::opencl>& bases() const;
   inline size_t colorSize(size_t color) const;
   inline size_t maxSize() const;
   inline size_t maxContacts(size_t color) const;
   inline size_t reductionSteps(size_t color) const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void init( size_t colors, size_t bodies );
   void calcOffsets();
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t bodies_;                        //!< The number of bodies in the current contact problem.
   size_t colors_;                        //!< The number of colors in the current contact problem.
   size_t maxSize_;                       //!< The maximum number of overall contacts of the same color.
   std::vector<size_t> involved_;         /*!< The number of contacts of a color c next to a
                                               non-fixed body b. The values corresponding to a
                                               tuple (c, b) are stored in row-major format. */
   std::vector<size_t> offsets_;          /*!< The index of the next unused offset in the update
                                               cache for a body b and a color c. The values
                                               corresponding to a tuple (c, b) are stored in
                                               row-major format. */
   std::vector<size_t> colorSizes_;       /*!< The number of overall contacts of a color c times 2
                                               minus the number of collisions of color c and
                                               involving a fixed body. */
   std::vector<size_t> maxContacts_;      /*!< The maximum number of contacts of a color c next to
                                               a non-fixed body b. The values corresponding to a
                                               tuple (c, b) are stored in row-major format. */
   std::vector<size_t> reductionSteps_;   /*!< The number of reduction steps necessary in the
                                               update cache for a color. */
   gpusolve::memory::Vector<uint32_t, gpusolve::interface::host> bases_; /*< The index of the first
                                               contact's offset in the update cache next to a
                                               non-fixed body b with a color c. The values
                                               corresponding to a tuple (c, b) are stored in
                                               row-major format. */
   gpusolve::memory::Vector<uint32_t, gpusolve::interface::opencl> basesOpenCL_; /*< The index of
                                               the first contact's offset in the update cache next
                                               to a non-fixed body b with a color c on the active
                                               OpenCL device. The values corresponding to a tuple
                                               (c, b) are stored in row-major format. */
   //@}
   //**********************************************************************************************
};




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the OpenCLColoredContacts class.
 */
inline OpenCLColoredContacts::OpenCLColoredContacts()
   : bodies_(0)
   , colors_(0)
   , maxSize_(0)
   , involved_()
   , offsets_()
   , colorSizes_()
   , maxContacts_()
   , reductionSteps_()
   , bases_()
   , basesOpenCL_()
{
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the number of contacts of the given color next to the given body.
 *
 * \param body The index of the body.
 * \param color The color index of the contacts.
 * \return The number of contacts of the given color next to the given body.
 */
inline size_t OpenCLColoredContacts::involved( size_t body, size_t color ) const {
   return involved_[color * bodies_ + body];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns a read-write reference to the number of contacts of the given color next to the
 *        given body.
 *
 * \param body The index of the body.
 * \param color The color index of the contacts.
 * \return A reference to the number of contacts of the given color next to the given body.
 */
inline size_t& OpenCLColoredContacts::involved( size_t body, size_t color ) {
   return involved_[color * bodies_ + body];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the index of the next unused offset in the update cache for the given body and
 *        color.
 *
 * \param body The index of the body.
 * \param color The color index of the contacts.
 * \return The index of the next unused offset in the update cache for the given body and color.
 */
inline size_t OpenCLColoredContacts::offset( size_t body, size_t color ) const {
   return offsets_[color * bodies_ + body];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns a read-write reference to the index of the next unused offset in the update cache
 *        for the given body and color.
 *
 * \param body The index of the body.
 * \param color The color index of the contacts.
 * \return A reference to the index of the next unused offset in the update cache for the given
 *         body and color.
 *
 * The offset is used and updated when building the reduction information.
 */
inline size_t& OpenCLColoredContacts::offset( size_t body, size_t color ) {
   return offsets_[color * bodies_ + body];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the index of the first contact's offset in the update cache next to the given
 *        body with the given color.
 *
 * \param body The index of the body.
 * \param color The color index of the contact.
 * \return The index of the first contact's offset in the update cache next to the given
 *         body with the given color.
 */
inline size_t OpenCLColoredContacts::base( size_t body, size_t color ) const {
   return bases_[color * bodies_ + body];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns a reference to a vector allocated on the active OpenCL device containing the
 *        basis information as returned by base().
 *        body with the given color.
 *
 * \return A reference to a vector allocated on the active OpenCL device containing the
 *         basis information as returned by base().
 */
inline const gpusolve::memory::Vector<cl_uint, gpusolve::interface::opencl>& OpenCLColoredContacts::bases() const {
   return basesOpenCL_;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the number of overall contacts of the same given color.
 *
 * \param color The color index of the contacts.
 * \return The maximum number of overall contacts of the same color.
 */
inline size_t OpenCLColoredContacts::colorSize( size_t color ) const {
   return colorSizes_[color];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the maximum number of overall contacts of the same color.
 *
 * \return The maximum number of overall contacts of the same color.
 *
 * It is computed in calcOffsets() as \f$\max_{c \in C} colorSize(c)\f$.
 */
inline size_t OpenCLColoredContacts::maxSize() const {
   return maxSize_;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the maximum number of contacts of the same given color next to any body.
 *
 * \param color The color index of the contacts.
 * \return The maximum number of contacts of the same given color next to any body.
 *
 * It is computed in calcOffsets() as \f$\max_{b \in B} involved(b, color)\f$.
 */
inline size_t OpenCLColoredContacts::maxContacts( size_t color ) const {
   return maxContacts_[color];
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns the number of reduction steps necessary in the update cache for a given color.
 *
 * \param color The color index of the contacts.
 * \return The number of reduction steps necessary in the update cache for a given color.
 *
 * It is computed in calcOffsets() as \f$\lceil \log_2 maxContacts(color) \rceil\f$.
 */
inline size_t OpenCLColoredContacts::reductionSteps( size_t color ) const {
   return reductionSteps_[color];
}
//*************************************************************************************************


//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resizes the data structures to accommodate the specified number of colors and bodies.
 *
 * \param colors The number of colors.
 * \param bodies The number of bodies.
 * \return void
 */
inline void OpenCLColoredContacts::init( size_t colors, size_t bodies ) {
   colors_ = colors;
   bodies_ = gpusolve::util::math::ceil<32>( bodies );

   size_t n = colors_ * bodies_;

   involved_.resize       ( n );
   offsets_.resize        ( n );
   colorSizes_.resize     ( colors_ );
   maxContacts_.resize    ( colors_ );
   reductionSteps_.resize ( colors_ );
   bases_.resize          ( n );
   basesOpenCL_.resize    ( n );

   // reset
   std::fill( involved_.begin(), involved_.end(), 0 );
   maxSize_ = 0;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Builds all attributes according to the information in involved_.
 *
 * \return void
 */
inline void OpenCLColoredContacts::calcOffsets() {
   for( size_t c = 0; c < colors_; ++c ) {
      size_t off = 0;
      for( size_t b = 0; b < bodies_; ++b ) {
         bases_[c * bodies_ + b] = off;
         offsets_[c * bodies_ + b] = off;
         off += involved_[c * bodies_ + b];
         maxContacts_[c] = std::max( maxContacts_[c], involved_[c * bodies_ + b] );
      }
      colorSizes_[c] = off;
      maxSize_ = std::max( maxSize_, off );
      reductionSteps_[c] = std::ceil( std::log10( maxContacts_[c] ) / std::log10( 2 ) );

      pe_LOG_DEBUG_SECTION( log ) {
         std::cerr << "cache size of color " << c << ": " << off << std::endl;
         std::cerr << "reduction steps for color " << c << ": " << reductionSteps_[c] << std::endl;
      }
   }

   basesOpenCL_ = bases_;
}
//*************************************************************************************************

} // namespace pe

#endif
