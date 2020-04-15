//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/SweepAndPrune.h
 *  \brief Implementation of the 'Sweep and Prune' coarse collision detection algorithm
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

#ifndef _PE_CORE_DETECTION_COARSE_SWEEPANDPRUNE_H_
#define _PE_CORE_DETECTION_COARSE_SWEEPANDPRUNE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <iosfwd>
#include <list>
#include <utility>
#include <vector>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/detection/coarse/IntervalEndpoint.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/Types.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the 'Sweep-And-Prune' coarse collision detection algorithm.
 * \ingroup coarse_collision_detection
 *
 * TODO: description
 * TODO: acknowledgements to OpenTissue
 *
 * \image html sweepandprune.png
 * \image latex sweepandprune.eps "The Sweep-And-Prune algorithm" width=420pt
 */
template< typename C >  // Type of the configuration
class SweepAndPrune : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                              Config;        //!< Type of the configuration.
   typedef SweepAndPrune<Config>          This;          //!< Type of this SweepAndPrune instance.
   typedef BodyStorage<Config>            BS;            //!< Type of the body storage.
   typedef typename Config::BodyID        BodyID;        //!< Handle for a rigid body.
   typedef typename Config::FineDetector  FineDetector;  //!< Type of the fine collision detector.
   //**********************************************************************************************

private:
   //**Private struct Collision********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Auxiliary class for a currently active collision between two rigid bodies. */
   struct Collision
   {
      //**Constructors*****************************************************************************
      /*!\name Constructor */
      //@{
      inline explicit Collision( BodyID body1, BodyID body2 );
      //@}
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      BodyID body1_;  //!< The first rigid body involved in the collision.
      BodyID body2_;  //!< The second rigid body involved in the collision.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   //! List of reported collisions between rigid bodies.
   typedef std::list<Collision>  Collisions;
   //**********************************************************************************************

   //**Private struct Counter**********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Overlap counter between two potentially colliding rigid bodies. */
   struct Counter
   {
      //**Constructors*****************************************************************************
      /*!\name Constructor */
      //@{
      explicit inline Counter();
      //@}
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool active_;                              //!< Activity flag value.
                                                 /*!< \a true in case there is an active collision
                                                      between two rigid bodies, \a false if not. */
      unsigned short counter_;                   //!< Overlap counter value.
      typename Collisions::iterator collision_;  //!< Reference to the currently active collision.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   //! Vector of projected interval endpoints of the bounding boxes of rigid bodies.
   /*! This data structure represents the x-, y- and z-coordinate axes that are used to
       store the projections of the start and endpoints of the axis-aligned bounding
       boxes of all rigid bodies. */
   typedef std::vector<IntervalEndpoint*>  CoordinateAxis;

   //! Pair of potentially colliding rigid bodies.
   /*! This data structure is used as key for the overlap counter hash map. */
   typedef std::pair<BodyID,BodyID>  BodyPair;

   //! Hash map data structure for the overlap counters.
   /*! This hash data structure is used to store the overlap counters. It provides a constant
       access performance and a reasonable memory requirement in comparison to other approaches
       to store the overlap counters. */
   typedef boost::unordered_map< BodyPair, Counter, boost::hash<BodyPair> >  HashMap;
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SweepAndPrune( BS& bodystorage );
   explicit SweepAndPrune( BS& bodystorage, BS& bodystorageShadowCopies );
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
   template< typename Contacts >
   void findContacts( Contacts& contacts );

   void clear();
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
          void   sort      ( CoordinateAxis& axis );
   inline bool   isWrong   ( const IntervalEndpoint* left, const IntervalEndpoint* right ) const;
          void   swapAction( const IntervalEndpoint* left, const IntervalEndpoint* right );
   inline BodyID majorBody ( BodyID body1, BodyID body2 ) const;
   inline BodyID minorBody ( BodyID body1, BodyID body2 ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void printAxis   ( std::ostream& os, const CoordinateAxis& axis ) const;
   void printHashMap( std::ostream& os )                             const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   CoordinateAxis xaxis_;   //!< Vector for the projections on the x-axis.
   CoordinateAxis yaxis_;   //!< Vector for the projections on the y-axis.
   CoordinateAxis zaxis_;   //!< Vector for the projections on the z-axis.
   Collisions collisions_;  //!< The reported collisions.
   HashMap hashmap_;        //!< Overlap counter storage.
   BS& bodystorage_;        //!< Reference to the central body storage.
   BS& bodystorageShadowCopies_;  //!< Reference to the body storage containing body shadow copies.
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
/*!\brief Constructor for the SweepAndPrune class.
 *
 * \param bodystorage Reference to the general body storage.
 *
 * Note that all (local, global and remote) must be contained in the body storage.
 */
template< typename Config >  // Type of the configuration
SweepAndPrune<Config>::SweepAndPrune( BS& bodystorage )
   : xaxis_()       // Vector for the projections on the x-axis
   , yaxis_()       // Vector for the projections on the y-axis
   , zaxis_()       // Vector for the projections on the z-axis
   , collisions_()  // The reported collisions
   , hashmap_()     // Overlap counter storage
   , bodystorage_( bodystorage )
   , bodystorageShadowCopies_( bodystorage )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the SweepAndPrune class.
 *
 * \param bodystorage Reference to the central body storage.
 * \param bodystorageShadowCopies Reference to the body storage containing all body shadow copies.
 */
template< typename Config >  // Type of the configuration
SweepAndPrune<Config>::SweepAndPrune( BS& bodystorage, BS& bodystorageShadowCopies )
   : xaxis_()       // Vector for the projections on the x-axis
   , yaxis_()       // Vector for the projections on the y-axis
   , zaxis_()       // Vector for the projections on the z-axis
   , collisions_()  // The reported collisions
   , hashmap_()     // Overlap counter storage
   , bodystorage_( bodystorage )
   , bodystorageShadowCopies_( bodystorageShadowCopies )
{}
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
template< typename Config >  // Type of the configuration
inline void SweepAndPrune<Config>::add( BodyID body )
{
   // Adding the interval endpoints to the coordinate axes
   xaxis_.push_back( &body->aabb_[0] );
   xaxis_.push_back( &body->aabb_[3] );
   yaxis_.push_back( &body->aabb_[1] );
   yaxis_.push_back( &body->aabb_[4] );
   zaxis_.push_back( &body->aabb_[2] );
   zaxis_.push_back( &body->aabb_[5] );

   // Sorting the three coordinate axes
   sort( xaxis_ );
   sort( yaxis_ );
   sort( zaxis_ );
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
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::remove( BodyID body )
{
   typedef typename BS::Iterator  Iterator;

   // Removing all endpoint projections from the coordinate axes
   // Note that at this point the axes cannot be assumed to be sorted. Therefore the 'std::find'
   // function is used instead of the 'std::lower_bound' function.
   CoordinateAxis::iterator pos;

   pos = std::find( xaxis_.begin(), xaxis_.end(), &body->aabb_[0] );
   pe_INTERNAL_ASSERT( pos != xaxis_.end(), "Start of x-interval is not contained in the x-axis" );
   pos = xaxis_.erase( pos );

   pos = std::find( pos, xaxis_.end(), &body->aabb_[3] );
   pe_INTERNAL_ASSERT( pos != xaxis_.end(), "End of x-interval is not contained in the x-axis" );
   xaxis_.erase( pos );

   pos = std::find( yaxis_.begin(), yaxis_.end(), &body->aabb_[1] );
   pe_INTERNAL_ASSERT( pos != yaxis_.end(), "Start of y-interval is not contained in the y-axis" );
   pos = yaxis_.erase( pos );

   pos = std::find( pos, yaxis_.end(), &body->aabb_[4] );
   pe_INTERNAL_ASSERT( pos != yaxis_.end(), "End of y-interval is not contained in the y-axis" );
   yaxis_.erase( pos );

   pos = std::find( zaxis_.begin(), zaxis_.end(), &body->aabb_[2] );
   pe_INTERNAL_ASSERT( pos != zaxis_.end(), "Start of z-interval is not contained in the z-axis" );
   pos = zaxis_.erase( pos );

   pos = std::find( pos, zaxis_.end(), &body->aabb_[5] );
   pe_INTERNAL_ASSERT( pos != zaxis_.end(), "End of z-interval is not contained in the z-axis" );
   zaxis_.erase( pos );

   // Removing all overlap counters from the hash map
   // Note that the approach to erase all possible keys containing the rigid body to be
   // deregistered is by far more efficient than iterating over all elements of the hash
   // map and erasing all elements actually containing the given rigid body.
   {
      const Iterator end( bodystorage_.end() );
      for( Iterator b=bodystorage_.begin(); b!=end; ++b ) {
         hashmap_.erase( BodyPair( body, *b ) );
         hashmap_.erase( BodyPair( *b, body ) );
      }
   }

   if( &bodystorage_ != &bodystorageShadowCopies_ ) {
      const Iterator end( bodystorageShadowCopies_.end() );
      for( Iterator b=bodystorageShadowCopies_.begin(); b!=end; ++b ) {
         hashmap_.erase( BodyPair( body, *b ) );
         hashmap_.erase( BodyPair( *b, body ) );
      }
   }

   // Removing all active collisions for the rigid body
   for( typename Collisions::iterator it=collisions_.begin(); it!=collisions_.end(); ) {
      if( it->body1_ == body || it->body2_ == body ) {
         it = collisions_.erase( it );
      }
      else ++it;
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
template< typename Config >    // Type of the configuration
template< typename Contacts >  // Contact container type
void SweepAndPrune<Config>::findContacts( Contacts& contacts )
{
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Finding the contacts via the sweep-and-prune algorithm...";
   }

   // Early exit
   pe_INTERNAL_ASSERT( xaxis_.size() == yaxis_.size(), "Invalid axis size detected" );
   pe_INTERNAL_ASSERT( xaxis_.size() == zaxis_.size(), "Invalid axis size detected" );
   if( xaxis_.size() < 2 ) return;

   // Sorting the three coordinate axes
   sort( xaxis_ );
   sort( yaxis_ );
   sort( zaxis_ );

   // Contact generation
   const typename Collisions::iterator begin( collisions_.begin() );
   const typename Collisions::iterator end  ( collisions_.end()   );

   for( typename Collisions::iterator it=begin; it!=end; ++it ) {
      if( !it->body1_->isFixed() || !it->body2_->isFixed() )  // Ignoring contacts between two fixed bodies
         FineDetector::collide( it->body1_, it->body2_, contacts );
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( contacts.isEmpty() )
         log << "      no contacts found!";
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
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::clear()
{
   // Clearing the coordinate axes
   xaxis_.clear();
   yaxis_.clear();
   zaxis_.clear();

   // Clearing all detected collisions
   collisions_.clear();

   // Clearing the overlap counters
   hashmap_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sorting of the given axis \a axis of projected bounding box intervals.
 *
 * \param axis The coordinate axis to be sorted.
 * \return void
 *
 * This function performes the sorting of a coordinate axis via the insertion sort sorting
 * algorithm and registers/deregisters collisions accordingly.
 */
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::sort( CoordinateAxis& axis )
{
   typedef typename CoordinateAxis::iterator  Iterator;

   const Iterator begin( axis.begin() );
   const Iterator end  ( axis.end()   );

   Iterator scan = begin;
   Iterator mark = scan+1;

   while( mark != end )
   {
      if( isWrong( *scan, *mark ) )
      {
         Iterator left ( scan  );
         Iterator right( mark );

         do {
            swapAction( *left, *right );
            std::iter_swap( right, left );
            if( left == begin ) break;
            --right;
            --left;
         }
         while( isWrong( *left, *right ) );
      }

      ++scan;
      ++mark;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Check of two adjacent interval endpoints.
 *
 * \param left The left-hand side interval endpoint.
 * \param right The right-hand side interval endpoint.
 * \return \a true in case \a left and \a right are in the wrong order, \a false if not.
 *
 * This function evaluates the sorting of the two adjacent interval endpoints. In case
 * the two endpoints are sorted wrong, the function returns \a true, otherwise \a false
 * is returned.
 */
template< typename Config >  // Type of the configuration
inline bool SweepAndPrune<Config>::isWrong( const IntervalEndpoint* left,
                                            const IntervalEndpoint* right ) const
{
   // The interval endpoints must be sorted in increasing order from left to right
   if( right->value_ < left->value_ )
      return true;

   // 'start' values must be sorted in front of 'end' values
   if( right->value_ == left->value_            &&
       right->type_  == IntervalEndpoint::start &&
       left->type_   == IntervalEndpoint::end ) {
      return true;
   }

   // No error detected
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param left The left-hand side interval endpoint to be swapped.
 * \param right The right-hand side interval endpoint to be swapped.
 * \return void
 *
 * This function is called whenever two interval endpoints are swapped by the insertion sort
 * algorithm. It performs the semantically necessary actions corresponding to the swap.
 * TODO
 */
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::swapAction( const IntervalEndpoint* left,
                                        const IntervalEndpoint* right )
{
   // Increasing the overlap counter for a newly detected interval overlap
   if( left->type_ == IntervalEndpoint::end && right->type_ == IntervalEndpoint::start )
   {
      const BodyID body1( majorBody( left->body_, right->body_ ) );
      const BodyID body2( minorBody( left->body_, right->body_ ) );

      Counter& counter( hashmap_[ BodyPair( body1, body2 ) ] );

      pe_INTERNAL_ASSERT( counter.counter_ < 3, "Invalid counter value" );
      ++counter.counter_;

      if( counter.counter_ == 3 ) {
         counter.collision_ = collisions_.insert( collisions_.end(), Collision( body1, body2 ) );
         counter.active_    = true;
      }
   }

   // Decreasing the overlap counter for two separating intervals
   else if( left->type_ == IntervalEndpoint::start && right->type_ == IntervalEndpoint::end )
   {
      const BodyID body1( majorBody( left->body_, right->body_ ) );
      const BodyID body2( minorBody( left->body_, right->body_ ) );

      Counter& counter( hashmap_[ BodyPair( body1, body2 ) ] );

      pe_INTERNAL_ASSERT( counter.counter_ > 0, "Invalid counter value" );
      --counter.counter_;

      if( counter.counter_ == 2 && counter.active_ ) {
         collisions_.erase( counter.collision_ );
         counter.active_ = false;
      }
      else if( counter.counter_ == 0 ) {
         hashmap_.erase( BodyPair( body1, body2 ) );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the major body of a pair of overlapping rigid bodies.
 *
 * \param body1 The first rigid body of the pair.
 * \param body2 The second rigid body of the pair.
 * \return The major rigid body.
 *
 * This function returns the major body of a pair of overlapping rigid bodies that is used to
 * store the overlap counter.
 */
template< typename Config >  // Type of the configuration
inline typename SweepAndPrune<Config>::BodyID
   SweepAndPrune<Config>::majorBody( BodyID body1, BodyID body2 ) const
{
   if( body1->getSystemID() < body2->getSystemID() ) return body1;
   else return body2;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the minor body of a pair of overlapping rigid bodies.
 *
 * \param body1 The first rigid body of the pair.
 * \param body2 The second rigid body of the pair.
 * \return The minor rigid body.
 *
 * This function returns the minor body of a pair of overlapping rigid bodies that is NOT used
 * to store the overlap counter.
 */
template< typename Config >  // Type of the configuration
inline typename SweepAndPrune<Config>::BodyID
   SweepAndPrune<Config>::minorBody( BodyID body1, BodyID body2 ) const
{
   if( body1->getSystemID() < body2->getSystemID() ) return body2;
   else return body1;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output function for a single coordinate axis.
 *
 * \param os Reference to the output stream.
 * \param axis The coordinate axis to be printed.
 * \return void
 */
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::printAxis( std::ostream& os, const CoordinateAxis& axis ) const
{
   os << "\n Sweep-and-prune coordinate axis\n";

   const typename CoordinateAxis::const_iterator begin( axis.begin() );
   const typename CoordinateAxis::const_iterator end  ( axis.end()   );

   for( typename CoordinateAxis::const_iterator it=begin; it!=end; ++it ) {
      os << " " << (*it)->value_ << " (body=" << (*it)->body_->getID();
      if( (*it)->type_ == IntervalEndpoint::start )
         os << ", start)\n";
      else
         os << ", end)\n";
   }

   os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output function for the overlap counter hash map.
 *
 * \return void
 */
template< typename Config >  // Type of the configuration
void SweepAndPrune<Config>::printHashMap( std::ostream& os ) const
{
   os << "\n Sweep-and-prune hash map\n";

   const typename HashMap::const_iterator begin( hashmap_.begin() );
   const typename HashMap::const_iterator end  ( hashmap_.end()   );

   for( typename HashMap::const_iterator it=begin; it!=end; ++it ) {
      const BodyPair& bp ( it->first  );
      const Counter&  cnt( it->second );
      os << " body1=" << bp.first->getID() << ", body2=" << bp.second->getID()
         << ": counter=" << cnt.counter_ << "\n";
   }

   os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS SWEEPANDPRUNE::COLLISION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Default constructor for the SweepAndPrune::Collision class.
 */
template< typename Config >  // Type of the configuration
inline SweepAndPrune<Config>::Collision::Collision( BodyID body1, BodyID body2 )
   : body1_( body1 )  // The first rigid body involved in the collision
   , body2_( body2 )  // The second rigid body involved in the collision
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS SWEEPANDPRUNE::COUNTER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Default constructor for the SweepAndPrune::Counter class.
 */
template< typename Config >  // Type of the configuration
inline SweepAndPrune<Config>::Counter::Counter()
   : active_( false )  // Activity flag value
   , counter_( 0 )     // Overlap counter value
   , collision_()      // Reference to the currently active collision
{}
/*! \endcond */
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
