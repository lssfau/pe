//=================================================================================================
/*!
 *  \file pe/core/batches/JointTrait.h
 *  \brief Header file for the JointTrait class
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

#ifndef _PE_CORE_BATCHES_JOINTTRAIT_H_
#define _PE_CORE_BATCHES_JOINTTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/batches/BodyTrait.h>
#include <pe/core/batches/Types.h>
#include <pe/util/Types.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Joint customization class for the batch generation.
 * \ingroup batch_generation
 *
 * The batch generation JointTrait class template is used to adapt the Joint class to
 * the used batch generation algorithm. Depending on the used algorithm, a joint requires
 * additional data or functionality to efficiently support the batch generation process.\n
 * In order to adapt the Joint class to a particular algorithm, the base template needs
 * to be specialized. The following members are expected for each specialization of the
 * JointTrait class template:
 *
 *  - the 'mergeNodes()' function
 */
template< typename C >  // Type of the configuration
class JointTrait
{
public:
   //**Type definitions****************************************************************************
   typedef BodyTrait<C>   Node;         //!< Type of the corresponding BodyTrait class.
   typedef Node*          NodeID;       //!< Handle to a Node object.
   typedef const Node*    ConstNodeID;  //!< Handle to a constant Node object.
   typedef JointTrait<C>  Edge;         //!< Type of this JointTrait instance.
   //**********************************************************************************************

   //**Edge functions******************************************************************************
   /*!\name Edge functions */
   //@{
   inline void mergeNodes( ConstNodeID node1, ConstNodeID node2 ) const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging two adjacent joint graphs.
 *
 * \param node1 The first attached node.
 * \param node2 The second attached node.
 * \return void
 *
 * Default implementation of the mergeNodes() function.
 */
template< typename C >  // Type of the configuration
inline void JointTrait<C>::mergeNodes( ConstNodeID /*node1*/, ConstNodeID /*node2*/ ) const
{}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE UNION FIND ALGORITHM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the JointTrait class template for the UnionFind algorithm.
 * \ingroup batch_generation
 *
 * This specialization of the JointTrait class template adapts rigid bodies to the 'Union Find'
 * batch generation algorithm. In this algorithm, rigid bodies act as nodes in a graph and
 * joints between the rigid bodies act as edges.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct JointTrait< C<CD,FD,UnionFind,CR> >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,UnionFind,CR>  Config;       //!< Type of the actual configuration.
   typedef BodyTrait<Config>      Node;         //!< Type of the corresponding BodyTrait class.
   typedef Node*                  NodeID;       //!< Handle to a Node object.
   typedef const Node*            ConstNodeID;  //!< Handle to a constant Node object.
   typedef JointTrait<Config>     Edge;         //!< Type of this JointTrait specialization.
   //**********************************************************************************************

   //**Edge functions******************************************************************************
   /*!\name Edge functions */
   //@{
   inline void mergeNodes( ConstNodeID node1, ConstNodeID node2 ) const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging two adjacent joint graphs.
 *
 * \param node1 The first attached node.
 * \param node2 The second attached node.
 * \return void
 *
 * This function merges the two joint graphs containing \a node1 and \a body2 into a single
 * joint graph.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void JointTrait< C<CD,FD,UnionFind,CR> >::mergeNodes( ConstNodeID node1, ConstNodeID node2 ) const
{
   const ConstNodeID root1( node1->getRoot() );
   const ConstNodeID root2( node2->getRoot() );

   if( root1 == root2 ) {
      return;
   }
   else if( root1->rank_ > root2->rank_ ) {
      root2->root_ = root1;
   }
   else if( root2->rank_ > root1->rank_ ) {
      root1->root_ = root2;
   }
   else {
      root2->root_ = root1;
      ++root1->rank_;
   }
}
//*************************************************************************************************

}  // namespace batches

} // namespace pe

#endif
