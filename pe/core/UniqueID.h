//=================================================================================================
/*!
 *  \file pe/core/UniqueID.h
 *  \brief Generation of unique IDs in MPI environments
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

#ifndef _PE_CORE_UNIQUEID_H_
#define _PE_CORE_UNIQUEID_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/MPISettings.h>
#include <pe/util/Assert.h>
#include <pe/util/NonCreatable.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper class for the UniqueID class template.
 * \ingroup core
 *
 * The UniqueIDTrait class provides the necessary byte-depending values for the generation of
 * unique IDs by the UniqueID class template.
 */
template< size_t N > struct UniqueIDTrait    {};
template<>           struct UniqueIDTrait<4> { enum { shift = 20 }; };
template<>           struct UniqueIDTrait<8> { enum { shift = 40 }; };
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Generation of unique IDs in MPI environments.
 * \ingroup core
 *
 * \image html uniqueid.png
 * \image latex uniqueid.eps "Unique ID generation" width=320pt
 *
 * The UniqueID class is responsible for the generation of unique system IDs even during an MPI
 * parallel execution. The unqiue ID is composed from the number of locally generated objects
 * of type \a T and the rank of the local MPI process.
 */
template< typename T >
class UniqueID : private NonCreatable
{
public:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static inline id_t create();
   static inline id_t createGlobal();
   //@}
   //**********************************************************************************************
   
   static inline id_t count() { return counter_; }
   static inline id_t globalCount() { return globalCounter_; }

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static id_t counter_;        //!< System ID counter.
   static id_t globalCounter_;  //!< Global system ID counter.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class BodyBinaryWriter;
   friend class BodyBinaryReader;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

template< typename T >
id_t UniqueID<T>::counter_( 0 );

template< typename T >
id_t UniqueID<T>::globalCounter_( 0 );




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Generates a unique ID.
 *
 * \return The newly generated unique ID.
 */
template< typename T >
inline id_t UniqueID<T>::create()
{
   id_t id( counter_++ );

   if( MPISettings::bits() > 0 ) {
      const size_t shift( sizeof(id_t)*size_t(8) - MPISettings::bits() );
      pe_INTERNAL_ASSERT( shift < sizeof(id_t)*size_t(8), "Invalid shift detected" );

      const int rank ( MPISettings::rank() );
      pe_INTERNAL_ASSERT( rank >= 0, "MPI system is not initialized" );

      if( counter_ >= ( id_t(1) << shift ) ) {
         // Revert overflow of counter
         --counter_;
         throw std::runtime_error( "Unable to create unique ID" );
      }

      id |= ( static_cast<id_t>( rank ) << shift );
   }
   else if( counter_ == size_t(0) ) {
      // Revert overflow of counter
      --counter_;
      throw std::runtime_error( "Unable to create unique ID" );
   }

   return id;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Generates an ID which is the same on all processes but unique on each process.
 *
 * \return The newly generated ID.
 */
template< typename T >
inline id_t UniqueID<T>::createGlobal()
{
   id_t id( globalCounter_++ );

   if( MPISettings::bits() > 0 ) {
      const size_t shift( sizeof(id_t)*size_t(8) - MPISettings::bits() );
      pe_INTERNAL_ASSERT( shift < sizeof(id_t)*size_t(8), "Invalid shift detected" );

      const int size( MPISettings::size() );
      pe_INTERNAL_ASSERT( size >= 0, "MPI system is not initialized" );

      if( globalCounter_ >= ( id_t(1) << shift ) ) {
         // Revert overflow of counter
         --globalCounter_;
         throw std::runtime_error( "Unable to create unique ID" );
      }

      id |= ( static_cast<id_t>( size ) << shift );
   }
   else if( globalCounter_ == size_t(0) ) {
      // Revert overflow of counter
      --globalCounter_;
      throw std::runtime_error( "Unable to create unique ID" );
   }

   return id;
}
//*************************************************************************************************

} // namespace pe

#endif
