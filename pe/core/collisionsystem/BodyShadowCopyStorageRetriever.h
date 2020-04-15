//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/BodyShadowCopyStorageRetriever.h
 *  \brief Class for retrieving body shadow copy storages.
 *
 *  Copyright (C) 2016 Tobias Preclik
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

#ifndef _PE_CORE_COLLISIONSYSTEM_BODYSHADOWCOPYSTORAGERETRIEVER_H_
#define _PE_CORE_COLLISIONSYSTEM_BODYSHADOWCOPYSTORAGERETRIEVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/CollisionSystem.h>




namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/**
 * Can be used to transparently handle old collision systems that do not provide a separate
 * shadow copy storage. In these cases the retriever returns an empty body storage.
 */
template<class C>
class BodyShadowCopyStorageRetriever
{
public:
   const BodyStorage<C>& retrieve() const
   {
      boost::shared_ptr< CollisionSystem<C> > cs( CollisionSystem<C>::instance() );
      return cs->getBodyShadowCopyStorage();
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::FFDSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::FFDSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::FFDSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::DEMSolverObsolete> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::DEMSolverObsolete> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::DEMSolverObsolete> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::OpenCLSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::OpenCLSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::OpenCLSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::FrictionlessSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::FrictionlessSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::FrictionlessSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::BoxFrictionSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::BoxFrictionSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::BoxFrictionSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::ConeFrictionSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::ConeFrictionSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::ConeFrictionSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************



//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BodyShadowCopyStorageRetriever< C<CD,FD,BG,response::PolyhedralFrictionSolver> >
{
public:
   const BodyStorage< C<CD,FD,BG,response::PolyhedralFrictionSolver> >& retrieve() const
   {
      static BodyStorage< C<CD,FD,BG,response::PolyhedralFrictionSolver> > bodyShadowCopyStorage;
      return bodyShadowCopyStorage;
   }
};
//*************************************************************************************************

} // namespace pe

#endif
