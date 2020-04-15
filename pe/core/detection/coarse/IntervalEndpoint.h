//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/IntervalEndpoint.h
 *  \brief Headerfile for an endpoint of an axis-aligned bounding box interval
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

#ifndef _PE_CORE_DETECTION_COARSE_INTERVALENDPOINT_H_
#define _PE_CORE_DETECTION_COARSE_INTERVALENDPOINT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <pe/core/Types.h>
#include <pe/math/MathTrait.h>
#include <pe/system/Precision.h>
#include <pe/util/Null.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Endpoint of an axis-aligned bounding box interval.
 * \ingroup coarse_collision_detection
 *
 * This class represents an endpoint for an axis-aligned bounding box interval. One instance
 * of this class represents either the start or the end of a bounding box interval on either
 * the x-, y- or z-axis of the global world frame.
 */
struct IntervalEndpoint
{
public:
   //**Types of interval endpoints*****************************************************************
   /*! Types of interval endpoints. */
   enum Type {
      invalid = 0,  //!< Type of an uninitialized endpoint.
      start   = 1,  //!< Type of the start of an interval.
      end     = 2   //!< Type of the end of an interval.
   };
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline IntervalEndpoint();
   explicit inline IntervalEndpoint( real value );
            inline IntervalEndpoint( const IntervalEndpoint& p );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   inline IntervalEndpoint& operator=( real value );
   inline IntervalEndpoint& operator=( const IntervalEndpoint& p );
   inline                   operator real() const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Type type_;    //!< The type of the interval endpoint.
   real value_;   //!< The current value of the interval endpoint.
   BodyID body_;  //!< The rigid body associated with this interval endpoint.
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
/*!\brief The default constructor for IntervalEndpoint.
 */
inline IntervalEndpoint::IntervalEndpoint()
   : type_(invalid)  // The type of the interval endpoint
   , value_(0)       // The current value of the interval endpoint
   , body_(NULL)     // The rigid body associated with this interval endpoint
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the direct initialization of the interval endpoint.
 *
 * \param value The current value of the interval endpoint.
 */
inline IntervalEndpoint::IntervalEndpoint( real value )
   : type_(invalid)  // The type of the interval endpoint
   , value_(value)   // The current value of the interval endpoint
   , body_(NULL)     // The rigid body associated with this interval endpoint
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for IntervalEndpoint.
 *
 * \param p The interval endpoint to be copied.
 */
inline IntervalEndpoint::IntervalEndpoint( const IntervalEndpoint& p )
   : type_(invalid)    // The type of the interval endpoint
   , value_(p.value_)  // The current value of the interval endpoint
   , body_(NULL)       // The rigid body associated with this interval endpoint
{}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the value of the interval endpoint.
 *
 * \param value The new value of the interval endpoint.
 * \return void
 */
inline IntervalEndpoint& IntervalEndpoint::operator=( real value )
{
   value_ = value;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for IntervalEndpoint.
 *
 * \param p The interval endpoint to be copied.
 * \return void
 */
inline IntervalEndpoint& IntervalEndpoint::operator=( const IntervalEndpoint& p )
{
   value_ = p.value_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion operator for access to the current value of the interval endpoint.
 *
 * \return The current value of the interval endpoint.
 */
inline IntervalEndpoint::operator real() const
{
   return value_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Interval endpoint operators */
//@{
inline std::ostream& operator<<( std::ostream& os, const IntervalEndpoint& p );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for interval endpoints.
 * \ingroup coarse_collision_detection
 *
 * \param os Reference to the output stream.
 * \param p Reference to a constant interval endpoint.
 * \return Reference to the output stream.
 */
inline std::ostream& operator<<( std::ostream& os, const IntervalEndpoint& p )
{
   return os << p.value_;
}
//*************************************************************************************************

} // namespace coarse

} // namespace detection




//=================================================================================================
//
//  MATHTRAIT SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
template< typename T >
struct MathTrait< T, detection::coarse::IntervalEndpoint >
{
   typedef typename MathTrait<T,real>::HighType  HighType;
   typedef typename MathTrait<T,real>::LowType   LowType;
   typedef typename MathTrait<T,real>::AddType   AddType;
   typedef typename MathTrait<T,real>::SubType   SubType;
   typedef typename MathTrait<T,real>::MultType  MultType;
   typedef typename MathTrait<T,real>::DivType   DivType;
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
template< typename T >
struct MathTrait< detection::coarse::IntervalEndpoint, T >
{
   typedef typename MathTrait<T,real>::HighType  HighType;
   typedef typename MathTrait<T,real>::LowType   LowType;
   typedef typename MathTrait<T,real>::AddType   AddType;
   typedef typename MathTrait<T,real>::SubType   SubType;
   typedef typename MathTrait<T,real>::MultType  MultType;
   typedef typename MathTrait<T,real>::DivType   DivType;
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
template<>
struct MathTrait< detection::coarse::IntervalEndpoint, detection::coarse::IntervalEndpoint >
{
   typedef detection::coarse::IntervalEndpoint  HighType;
   typedef detection::coarse::IntervalEndpoint  LowType;
   typedef detection::coarse::IntervalEndpoint  AddType;
   typedef detection::coarse::IntervalEndpoint  SubType;
   typedef detection::coarse::IntervalEndpoint  MultType;
   typedef detection::coarse::IntervalEndpoint  DivType;
};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
