//=================================================================================================
/*!
 *  \file pe/core/Serialization.h
 *  \brief Serialization of MPI processes in a parallel environment
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

#ifndef _PE_CORE_SERIALIZATION_H_
#define _PE_CORE_SERIALIZATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Serialization of MPI processes in a parallel environment.
 * \ingroup mpi
 *
 * The Serialization class is an auxiliary helper class for the \a pe_SERIALIZATION macro. It
 * is used as a loop counter in the serialization process and provides the synchronization of
 * the parallel MPI processes.
 */
class SerializationCounter
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline SerializationCounter();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   inline SerializationCounter& operator++();
   inline SerializationCounter  operator++( int );
   //@}
   //**********************************************************************************************

   //**Conversion operator*************************************************************************
   /*!\name Conversion operator */
   //@{
   inline operator int() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   int counter_;  //!< Value of the serialization counter.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for SerializationCounter.
 *
 * \exception std::runtime_error Invalid serialization inside exclusive section.
 */
inline SerializationCounter::SerializationCounter()
   : counter_( 0 )  // Value of the serialization counter
{
   // Checking if the function is called inside an exclusive section
   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
      throw std::runtime_error( "Invalid serialization inside exclusive section" );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Pre-increment operator.
 *
 * \return Reference to the incremented serialization counter.
 */
inline SerializationCounter& SerializationCounter::operator++()
{
   const int size( MPISettings::size() );

   if( size > 1 && counter_ < size )
      MPI_Barrier( MPISettings::comm() );

   ++counter_;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Post-increment operator.
 *
 * \return The incremented serialization counter.
 */
inline SerializationCounter SerializationCounter::operator++( int )
{
   const int size( MPISettings::size() );

   if( size > 1 && counter_ < size )
      MPI_Barrier( MPISettings::comm() );

   SerializationCounter tmp( *this );
   ++counter_;
   return tmp;
}
//*************************************************************************************************




//=================================================================================================
//
//  CONVERSION OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Conversion operator to \a int.
 *
 * The conversion operator returns the current value of the serialization counter.
 */
inline SerializationCounter::operator int() const
{
   return counter_;
}
//*************************************************************************************************








//=================================================================================================
//
//  SERIALIZATION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Serialization of MPI processes in a parallel environment.
 * \ingroup mpi
 *
 * This macro provides the option to serialize the parallel MPI processes. The following example
 * demonstrates how the serialization macro is used:

   \code
   int main( int argc, char** argv )
   {
      // Initialization of the MPI system
      // The MPI system must be initialized before any pe functionality is used. It is
      // recommended to make MPI_Init() the very first call of the main function.
      MPI_Init( &argc, &argv );

      // Aquiring the rank of the MPI process
      const int rank( theMPISystem()->getRank() );

      // Serialization of all MPI processes
      // The following section is executed by all MPI processes in ascending order. In other
      // words, the code contained within the serialized region is first executed by process
      // 0, followed by process 1, then process 2, and so on. Note that all other processes
      // are inactive!
      pe_SERIALIZATION {
         std::cout << " Process " << rank << std::endl;
      }

      // Finalization of the MPI system
      MPI_Finalize();
   }
   \endcode

 * Assuming that we are running the program from the example above using 4 MPI processes, the
 * output is as follows:

   \code
   Process 0
   Process 1
   Process 2
   Process 3
   \endcode

 * Note that in contrast to a parallel execution, the order of the output is deterministic.
 * However, also note that the code within a serialized section is executed serially with
 * an additional overhead for the MPI process synchronization.
 */
#define pe_SERIALIZATION \
   for( SerializationCounter i; i<MPISettings::size(); ++i ) \
      if( MPISettings::rank() == i )
//*************************************************************************************************

} // namespace pe

#endif
