//=================================================================================================
/*!
 *  \file pe/core/rigidbody/rigidbodytrait/HardContactSemiImplicitTimeSteppingSolvers.h
 *  \brief Specialization of the RigidBodyTrait class template for the hard contact solvers.
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_HARDCONTACTSEMIIMPLICITTIMESTEPPINGSOLVERS_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_HARDCONTACTSEMIIMPLICITTIMESTEPPINGSOLVERS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/MPIRigidBodyTrait.h>
#include <pe/core/response/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the RigidBodyTrait class template for the hard contact solvers.
// \ingroup rigid_body
//
// This specialization of the RigidBodyTrait class template adapts all rigid bodies to include
// attributes and members needed by the MPI parallel solver by inheriting from MPIRigidBodyTrait.
*/
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class RigidBodyTrait< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> > : public MPIRigidBodyTrait
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBodyTrait();
   //@}
   //**********************************************************************************************

public:

   size_t index_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait constructor.
//
// \param body The body ID of this rigid body.
*/
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
RigidBodyTrait< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::RigidBodyTrait( BodyID body )
   : MPIRigidBodyTrait( body )  // Initialization of the parent class
   , index_( 0 )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait destructor.
*/
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
RigidBodyTrait< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::~RigidBodyTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
