//=================================================================================================
/*!
 *  \file pe/core/ParallelTrait.h
 *  \brief Header file for the parallel trait
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

#ifndef _PE_CORE_PARALLELTRAIT_H_
#define _PE_CORE_PARALLELTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/response/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check of the MPI parallelism of the selected configuration.
 * \ingroup mpi
 *
 * The ParallelTrait class template tests whether the given configuration provides a MPI
 * parallel execution or not. In case a MPI parallel execution is possible, the \a value
 * member enumeration is set to 1. Otherwise it is set to 0. The enumeration can be used
 * at compile time in any constant expression.
 */
template< typename C >  // Type of the configuration
struct ParallelTrait
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = 0 };
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE FAST FRICTIONAL DYNAMICS SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the ParallelTrait class template for the fast frictional dynamics solver.
 * \ingroup mpi
 *
 * This specialization of the ParallelTrait class template provides the information that a MPI
 * parallel execution is possible for the fast frictional dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ParallelTrait< C<CD,FD,BG,response::FFDSolver> >
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE OLD DISCRETE ELEMENT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the ParallelTrait class template for the discrete element solver.
 * \ingroup mpi
 *
 * This specialization of the ParallelTrait class template provides the information that a MPI
 * parallel execution is possible for the discrete element solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ParallelTrait< C<CD,FD,BG,response::DEMSolverObsolete> >
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE DISCRETE ELEMENT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the ParallelTrait class template for the discrete element solver.
 * \ingroup mpi
 *
 * This specialization of the ParallelTrait class template provides the information that a MPI
 * parallel execution is possible for the discrete element solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ParallelTrait< C<CD,FD,BG,response::DEMSolver> >
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE HARD CONTACT SEMI-IMPLICIT TIMESTEPPING SOLVERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the ParallelTrait class template for the hard contact solvers.
 * \ingroup mpi
 *
 * This specialization of the ParallelTrait class template provides the information that an MPI
 * parallel execution is possible for the hard contact solvers.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ParallelTrait< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   //**********************************************************************************************
};

/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
