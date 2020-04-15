//=================================================================================================
/*!
 *  \file pe/util/TransferMeter.h
 *  \brief Transfer meter for analyzing the amount of transfered data.
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

#ifndef _PE_UTIL_TRANSFERMETER_H_
#define _PE_UTIL_TRANSFERMETER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <limits>
#ifdef __APPLE__
#include <malloc/malloc.h>
#else
#include <malloc.h>
#endif


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Transfer meter for analyzing the amount of transfered data.
 * \ingroup util
 */
class TransferMeter
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline TransferMeter();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Recording functions****************************************************************************
   /*!\name Recording functions */
   //@{
   inline void transfered( size_t bytes );
   inline void reset();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t getNumberOfTransfers() const;
   //@}
   //**********************************************************************************************

   //**Transfer evaluation functions*******************************************************************
   /*!\name Transfer evaluation functions */
   //@{
   inline size_t totalBytes() const;
   inline size_t minBytes()   const;
   inline size_t maxBytes()   const;
   //@}
   //**********************************************************************************************

private:
   size_t counterTransfers_;
   size_t totalBytes_;
   size_t minBytes_;
   size_t maxBytes_;
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the TransferMeter class.
 */
inline TransferMeter::TransferMeter()
   : counterTransfers_( 0 )
   , totalBytes_( 0 )
   , minBytes_( std::numeric_limits<size_t>::max() )
   , maxBytes_( 0 )
{}
//*************************************************************************************************




//=================================================================================================
//
//  RECORDING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Record a transfer.
 *
 * \param bytes The number of bytes transfered.
 * \return void
 */
inline void TransferMeter::transfered( size_t bytes )
{
   totalBytes_ += bytes;
   minBytes_ = std::min( minBytes_, bytes );
   maxBytes_ = std::max( maxBytes_, bytes );
   ++counterTransfers_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the meter.
 *
 * \return void
 */
inline void TransferMeter::reset()
{
   counterTransfers_ = 0;
   totalBytes_ = 0;
   minBytes_ = std::numeric_limits<size_t>::max();
   maxBytes_ = 0;
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the total number of transfers recorded.
 *
 * \return The number of recorded transfers.
 */
inline size_t TransferMeter::getNumberOfTransfers() const
{
   return counterTransfers_;
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSFER EVALUATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the total number of bytes transfered.
 *
 * \return The total number of bytes transfered.
 */
inline size_t TransferMeter::totalBytes() const
{
   return totalBytes_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the minimum number of bytes transfered in a single transfer.
 *
 * \return The minimum number of bytes transfered in a single transfer.
 */
inline size_t TransferMeter::minBytes() const
{
   return minBytes_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum number of bytes transfered in a single transfer.
 *
 * \return The maximum number of bytes transfered in a single transfer.
 */
inline size_t TransferMeter::maxBytes() const
{
   return maxBytes_;
}
//*************************************************************************************************

} // pe

#endif
