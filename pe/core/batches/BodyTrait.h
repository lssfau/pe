//=================================================================================================
/*!
 *  \file pe/core/batches/BodyTrait.h
 *  \brief Rigid body customization class for the batch generation
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

#ifndef _PE_CORE_BATCHES_BODYTRAIT_H_
#define _PE_CORE_BATCHES_BODYTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/batches/Types.h>
#include <pe/util/Types.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  ::pe::batches NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename C > class ContactTrait;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the batch generation.
 * \ingroup batch_generation
 *
 * The batch generation BodyTrait class template is used to adapt the RigidBody class to the
 * used batch generation algorithm. Depending on the used algorithm, a rigid body requires
 * additional data or functionality to efficiently support the batch generation process.\n
 * In order to adapt the RigidBody class to a particular algorithm, the base template needs
 * to be specialized. The following members are expected for each specialization of the
 * BodyTrait class template:
 *
 *  - the 'resetNode()' function
 */
template< typename C >  // Type of the configuration
class BodyTrait
{
protected:
   //**Node functions******************************************************************************
   /*!\name Node functions */
   //@{
   inline void resetNode() const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the data members of the node.
 *
 * \return void
 *
 * Default implementation of the resetNode() function.
 */
template< typename C >  // Type of the configuration
inline void BodyTrait<C>::resetNode() const
{}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE UNION FIND ALGORITHM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the BodyTrait class template for the 'Union Find' algorithm.
 * \ingroup batch_generation
 *
 * This specialization of the BodyTrait class template adapts rigid bodies to the 'Union Find'
 * batch generation algorithm. In this algorithm, rigid bodies act as nodes in a graph and
 * contacts between the rigid bodies act as edges.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyTrait< C<CD,FD,UnionFind,CR> >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,UnionFind,CR>  Config;       //!< Type of the actual configuration.
   typedef BodyTrait<Config>      Node;         //!< Type of this BodyTrait specialization.
   typedef Node*                  NodeID;       //!< Handle to a BodyTrait instance.
   typedef const Node*            ConstNodeID;  //!< Handle to a constant BodyTrait instance.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline BodyTrait();
   //@}
   //**********************************************************************************************

   //**Node functions******************************************************************************
   /*!\name Node functions */
   //@{
   inline ConstNodeID getRoot()   const;
   inline void        resetNode() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   mutable ConstNodeID root_;  //!< The root of the contact graph containing the rigid body.
   mutable size_t rank_;       //!< The current rank of the rigid body in the contact graph.
   mutable size_t batch_;      //!< Index of the batch containing all contacts in the contact graph.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class ContactTrait<Config>;
   template<typename> friend class UnionFind;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the BodyTrait<UnionFind> specialization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline BodyTrait< C<CD,FD,UnionFind,CR> >::BodyTrait()
   : root_ ( this )
   , rank_ ( 0 )
   , batch_( 0 )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the current root of the contact graph.
 *
 * \return The current root of the contact graph.
 *
 * This function returns the current root of the contact graph and compresses the graph for
 * a fast access to the root node.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename BodyTrait< C<CD,FD,UnionFind,CR> >::ConstNodeID
   BodyTrait< C<CD,FD,UnionFind,CR> >::getRoot() const
{
   ConstNodeID root( this );

   // Traverse the contact graph to find the root node
   while( root->root_ != root ) {
      root = root->root_;
   }

   // Retraverse the graph for graph compression
   ConstNodeID tmp( this );
   while( tmp->root_ != tmp ) {
      ConstNodeID last = tmp;
      tmp = tmp->root_;
      last->root_ = root;
   }

   return root;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the data members of the node.
 *
 * \return void
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void BodyTrait< C<CD,FD,UnionFind,CR> >::resetNode() const
{
   root_  = this;
   rank_  = 0;
   batch_ = 0;
}
//*************************************************************************************************

} // namespace batches

} // namespace pe

#endif
