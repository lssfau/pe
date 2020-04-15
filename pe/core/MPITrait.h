//=================================================================================================
/*!
 *  \file pe/core/MPITrait.h
 *  \brief Header file for the MPI trait
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

#ifndef _PE_CORE_MPITRAIT_H_
#define _PE_CORE_MPITRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPI.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\class MPITrait
 * \brief Base template for the MPITrait class..
 * \ingroup mpi
 *
 * The MPITrait class template offers a translation between the C++ built-in data types and
 * the corresponding MPI data types required for send and receive operations. For a particular
 * MPITrait instantiation, the corresponding MPI data type can be obtained via the \a type
 * member of the MPITrait. The following example demonstrates the application of the MPITrait
 * class:

   \code
   // Initialization of the MPI communication
   int* pi;  // Integer buffer for the MPI send operation
   ...       // Initialization of the send buffer

   // Sending 50 integers to process 0
   MPI_Send( pi, 50, MPITrait<int>::type, 0, 0, MPI_COMM_WORLD );
   \endcode
 */
template< typename T >
struct MPITrait;
//*************************************************************************************************




//=================================================================================================
//
//  MPITRAIT SPECIALIZATION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Macro for the creation of MPITrait specializations for the supported data types.
 * \ingroup mpi
 *
 * This macro is used for the setup of the MPITrait specializations for the data types
 * supported by MPI.
 */
#define pe_CREATE_MPITRAIT_SPECIALIZATION(CPP_TYPE,MPI_TYPE) \
   template<> \
   struct MPITrait< CPP_TYPE > \
   { \
      static inline MPI_Datatype getType() { return (MPI_TYPE); } \
   }
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  MPITRAIT SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
pe_CREATE_MPITRAIT_SPECIALIZATION( char              , MPI_CHAR               );
pe_CREATE_MPITRAIT_SPECIALIZATION( signed char       , MPI_SIGNED_CHAR        );
pe_CREATE_MPITRAIT_SPECIALIZATION( signed short int  , MPI_SHORT              );
pe_CREATE_MPITRAIT_SPECIALIZATION( signed int        , MPI_INT                );
pe_CREATE_MPITRAIT_SPECIALIZATION( signed long int   , MPI_LONG               );
pe_CREATE_MPITRAIT_SPECIALIZATION( signed long long  , MPI_LONG_LONG          );
pe_CREATE_MPITRAIT_SPECIALIZATION( unsigned char     , MPI_UNSIGNED_CHAR      );
pe_CREATE_MPITRAIT_SPECIALIZATION( unsigned short int, MPI_UNSIGNED_SHORT     );
pe_CREATE_MPITRAIT_SPECIALIZATION( unsigned int      , MPI_UNSIGNED           );
pe_CREATE_MPITRAIT_SPECIALIZATION( unsigned long int , MPI_UNSIGNED_LONG      );
pe_CREATE_MPITRAIT_SPECIALIZATION( unsigned long long, MPI_UNSIGNED_LONG_LONG );
pe_CREATE_MPITRAIT_SPECIALIZATION( float             , MPI_FLOAT              );
pe_CREATE_MPITRAIT_SPECIALIZATION( double            , MPI_DOUBLE             );
pe_CREATE_MPITRAIT_SPECIALIZATION( long double       , MPI_LONG_DOUBLE        );
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
