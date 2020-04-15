//=================================================================================================
/*!
 *  \file pe/core/response/ContactGraphColoring.h
 *  \brief Implementation of graph coloring algorithms.
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

#ifndef _PE_CORE_RESPONSE_CONTACTGRAPHCOLORING_H_
#define _PE_CORE_RESPONSE_CONTACTGRAPHCOLORING_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <map>
#include <vector>
#include <pe/util/Limits.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  FUNCTION DEFINITIONS
//
//=================================================================================================

/*!\defgroup coloring Coloring
 * \ingroup sparse_matrix
 */

//*************************************************************************************************
/*!\brief Approximative coloring of contact graphs.
 * \ingroup coloring
 *
 * \param contacts The contacts to color.
 * \param maxcolors The maximum number of colors to use. Default is unlimited.
 */
template< typename Contacts >  // Contact container type
std::vector<size_t> colorBalanced( const Contacts& contacts, size_t maxcolors = Limits<size_t>::inf() ) {
   typedef std::multimap<size_t, size_t>::iterator HeapIterator;

   const size_t n( contacts.size() );
   std::multimap<size_t, size_t, std::greater<size_t> > heap;
   std::vector<HeapIterator> pointers( n );
   std::vector<size_t> colors( n, Limits<size_t>::inf() );
   size_t maxdegree( 0 ), numcolors( 1 ), degree;

   for( size_t i = 0; i < n; ++i ) {
      const BodyID b1( contacts[i]->getBody1() );
      const BodyID b2( contacts[i]->getBody2() );

      pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(), "Invalid contact between two fixed bodies detected" );

      if( b1->isFixed() )
         degree = b2->countContacts();
      else if( b2->isFixed() )
         degree = b1->countContacts();
      else
         degree = b1->countContacts() + b2->countContacts() - 1;

      pointers[i] = heap.insert( std::make_pair( degree, i ) );
      maxdegree = max( maxdegree, degree );
   }

   std::vector<size_t> occurances;
   std::vector<size_t> colorcount( numcolors, 0 );

   while( !heap.empty() ) {
      size_t node = heap.begin()->second;
      heap.erase( heap.begin() );
      occurances.assign( numcolors, 0 );

      const BodyID b[2] = { contacts[node]->getBody1(), contacts[node]->getBody2() };

      for( int j = 0; j < 2; ++j) {
         const RigidBody::ContactIterator end( b[j]->endContacts() );

         for( RigidBody::ContactIterator i = b[j]->beginContacts(); i != end; ++i ) {
            size_t nb = i->getIndex();
            if( nb == node )
               continue;

            if( colors[nb] < numcolors ) {
               // exclude color
               ++occurances[ colors[nb] ];
            }
            else {
               // decrement degree
               pe_INTERNAL_ASSERT( pointers[nb]->second == nb, "Helper data structure is corrupted." );
               degree = pointers[nb]->first;
               if( pointers[nb] == heap.begin() ) {
                  heap.erase( pointers[nb] );
                  pointers[nb] = heap.insert( std::make_pair( degree - 1, nb ) );
               }
               else {
                  heap.erase( pointers[nb]-- );
                  pointers[nb] = heap.insert( pointers[nb], std::make_pair( degree - 1, nb ) );
               }
            }
         }
      }

      // the following causes the coloring complexity to be n*c but produces a well-balanced coloring
      size_t mincolor = 0;
      for( size_t i = 1; i < numcolors; ++i )
         if( occurances[i] < occurances[mincolor] || ( occurances[i] == occurances[mincolor] && colorcount[i] < colorcount[mincolor] ) )
            mincolor = i;

      if( occurances[mincolor] > 0 && numcolors < maxcolors ) {
         colors[node] = numcolors++;
         colorcount.resize( numcolors );
         colorcount[colors[node]] = 1;
      }
      else {
         colors[node] = mincolor;
         colorcount[colors[node]]++;
      }
   }

   return colors;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Approximative coloring of contact graphs
 * \ingroup coloring
 *
 * \param contacts The contacts to color.
 */
template< typename Contacts >  // Contact container type
std::vector<size_t> color( const Contacts& contacts ) {
   typedef std::multimap<size_t, size_t>::iterator HeapIterator;

   const size_t n( contacts.size() );
   std::multimap<size_t, size_t, std::greater<size_t> > heap;
   std::vector<HeapIterator> pointers( n );
   std::vector<size_t> colors( n, Limits<size_t>::inf() );
   size_t maxdegree( 0 ), numcolors( 1 ), degree;

   for( size_t i = 0; i < n; ++i ) {
      const BodyID b1( contacts[i]->getBody1() );
      const BodyID b2( contacts[i]->getBody2() );

      pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(), "Invalid contact between two fixed bodies detected" );

      if( b1->isFixed() )
         degree = b2->countContacts();
      else if( b2->isFixed() )
         degree = b1->countContacts();
      else
         degree = b1->countContacts() + b2->countContacts() - 1;

      pointers[i] = heap.insert( std::make_pair( degree, i ) );
      maxdegree = max( maxdegree, degree );
   }

   std::vector<size_t> occurances;
   occurances.reserve( maxdegree - 1 );
   std::vector<size_t> colorcount( numcolors, 0 );

   while( !heap.empty() ) {
      size_t node = heap.begin()->second;
      heap.erase( heap.begin() );
      occurances.clear();
      const BodyID b[2] = { contacts[node]->getBody1(), contacts[node]->getBody2() };

      for( int j = 0; j < 2; ++j) {
         const RigidBody::ContactIterator end( b[j]->endContacts() );

         for( RigidBody::ContactIterator i = b[j]->beginContacts(); i != end; ++i ) {
            size_t nb = i->getIndex();
            if( nb == node )
               continue;

            if( colors[nb] < numcolors ) {
               // exclude color
               occurances.push_back( colors[nb] );
            }
            else {
               // decrement degree
               pe_INTERNAL_ASSERT( pointers[nb]->second == nb, "Helper data structure is corrupted." );
               degree = pointers[nb]->first;
               if( pointers[nb] == heap.begin() ) {
                  heap.erase( pointers[nb] );
                  pointers[nb] = heap.insert( std::make_pair( degree - 1, nb ) );
               }
               else {
                  heap.erase( pointers[nb]-- );
                  pointers[nb] = heap.insert( pointers[nb], std::make_pair( degree - 1, nb ) );
               }
            }
         }
      }

      // the following causes the coloring complexity to be n*log n but possibly produces a poorly-balanced coloring
      if( occurances.size() == 0 ) {
         // any color in [0; numcolors)
         colors[node] = numcolors - 1;
         colorcount[colors[node]]++;
      }
      else {
         // find color not in vector occurances
         std::sort( occurances.begin(), occurances.end() );
         std::iterator_traits< std::vector<size_t>::iterator >::difference_type tmp, begin = 0, last;
         last = std::distance( occurances.begin(), std::unique( occurances.begin(), occurances.end() ) ) - 1;

         while( begin != last ) {
            tmp = ( begin + last ) / 2;

            if( occurances[tmp] > size_t( tmp ) ) {
               // missing colors in [begin; tmp] (and possibly in [tmp+1; last])
               last = tmp;
            }
            else {
               // missing colors possibly in [tmp+1; last] but not in [begin; tmp]
               pe_INTERNAL_ASSERT( occurances[tmp] == size_t( tmp ), "Unused color search corrupted." );
               begin = tmp + 1;
            }
         }

         // since no colors are missing in range [occurances.begin(); begin) and begin == last and occurances[begin] > begin we now know that color "begin" is not in colors unless begin == end - 1.
         if( occurances[begin] > size_t( begin ) ) {
            colors[node] = begin;
            colorcount[colors[node]]++;
         }
         else {
            // no unused color in [0; end) (note that begin == last == end-1)
            if( size_t( begin + 1 ) < numcolors ) {
               // any color in [begin + 1; numcolors)
               colors[node] = numcolors - 1;
               colorcount[colors[node]]++;
            }
            else {
               // add new color
               colors[node] = numcolors++;
               colorcount.resize( numcolors );
               colorcount[colors[node]] = 1;
            }
         }
      }
   }

   return colors;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Reorder contact so that contacts of the same color are successively stored.
 * \ingroup coloring
 *
 * \param contacts The contact storage to reorder.
 * \param groups The color of each contact.
 * \return The number of contacts for each color.
 */
template< typename Contacts >  // Contact container type
std::vector<size_t> reorderContacts( Contacts& contacts, const std::vector<size_t>& groups ) {
   std::vector<size_t> groupSizes, groupOffsets;
   pe_USER_ASSERT( contacts.size() == groups.size(), "For each contact an associated group must be present." );

   for( std::vector<size_t>::const_iterator i = groups.begin(); i != groups.end(); ++i ) {
      if( size_t( *i ) >= groupSizes.size() )
         groupSizes.resize( *i + 1, 0 );
      ++groupSizes[*i];
   }

   const size_t n( groupSizes.size() );
   groupOffsets.resize( n + 1, 0 );
   for( size_t i = 0; i < n; ++i ) {
      groupOffsets[i + 1] = groupOffsets[i] + groupSizes[i];
      groupSizes[i] = 0;
   }

   size_t curGroup;
   for( size_t i = 0; i < n; ++i )
      for( size_t j = groupOffsets[i] + groupSizes[i]; j < groupOffsets[i + 1]; ++j ) {
         curGroup = groups[j];
         while( curGroup != i ) {
            const size_t dst( groupOffsets[curGroup] + groupSizes[curGroup]++ );
            std::swap( contacts[j], contacts[dst] );
            curGroup = groups[dst];
         }
         ++groupSizes[i];
      }

   // correct the batch index of the contacts
   for( size_t i = 0; i < contacts.size(); ++i )
      contacts[i]->setIndex( i );

   return groupSizes;
}
//*************************************************************************************************


} // namespace pe

#endif

