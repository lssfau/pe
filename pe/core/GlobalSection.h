//=================================================================================================
/*!
 *  \file pe/core/GlobalSection.h
 *  \brief Global section for the setup of global rigid bodies
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

#ifndef _PE_CORE_GLOBALSECTION_H_
#define _PE_CORE_GLOBALSECTION_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global section for the setup of global rigid bodies.
 * \ingroup mpi
 *
 * The GlobalSection class is an auxiliary helper class for the \a pe_GLOBAL_SECTION macro.
 * It provides the functionality to detect whether a global section is active, i.e. if the
 * currently executed code is inside a global section.
 */
class PE_PUBLIC GlobalSection
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   GlobalSection( bool activate );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~GlobalSection();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool isActive();
   //@}
   //**********************************************************************************************

   //**Conversion operator*************************************************************************
   /*!\name Conversion operator */
   //@{
   inline operator bool() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static bool active_;  //!< Activity flag for the global section.
                         /*!< In case a global section is active (i.e. the currently executed
                              code is inside a global section), the flag is set to \a true,
                              otherwise it is \a false. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor of the GlobalSection class.
 */
inline GlobalSection::~GlobalSection()
{
   active_ = false;  // Resetting the activity flag
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether a global section is active or not.
 *
 * \return \a true if a global section is active, \a false if not.
 */
inline bool GlobalSection::isActive()
{
   return active_;
}
//*************************************************************************************************




//=================================================================================================
//
//  CONVERSION OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Conversion operator to \a bool.
 *
 * The conversion operator returns \a true in case a global section is active and \a false
 * otherwise.
 */
inline GlobalSection::operator bool() const
{
   return active_;
}
//*************************************************************************************************








//=================================================================================================
//
//  GLOBAL SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global section for the setup of global rigid bodies.
 * \ingroup mpi
 *
 * This macro provides the option to start a global section for the setup of global rigid
 * bodies that are created on all processes. The following example demonstrates how a global
 * section is used:

   \code
   int main( int argc, char** argv )
   {
      // Initialization of the MPI system
      // The MPI system must be initialized before any pe functionality is used. It is
      // recommended to make MPI_Init() the very first call of the main function.
      MPI_Init( &argc, &argv );

      // Acquiring a handle to the simulation world
      WorldID world = theWorld();

      // Setup of the process connections
      ...

      // Default parallel region
      // This code is executed by all processes of the parallel simulation. Rigid bodies
      // created in this region may only be created if their center of mass lies within
      // the domain of the local process. In this example, the global position of the iron
      // sphere 1 with a radius of 1.2 is checked by the World::containsPoint() function
      // prior to its creation. Therefore the sphere is created on exactly one process.
      const Vec3 gpos( 2.0, 4.0, -3.0 );
      if( world->containsPoint( gpos ) ) {
         createSphere( 1, gpos, 1.2, iron );
      }

      // Global parallel region
      // The pe_GLOBAL_SECTION macro starts a global section for the setup of global rigid
      // bodies. Global rigid bodies are fixed bodies that are known on all MPI processes.
      // This feature can for instance be used to set up very large, fixed, process-spanning
      // bodies that act as simulation boundaries. Note that it is not possible to make a
      // global rigid body a non-global body or vice versa!
      pe_GLOBAL_SECTION
      {
         // Setup of a global sphere
         createSphere( 2, Vec3( 10.0, 20.0, -40.0 ), 5.0, oak );

         // Setup of a global union
         pe_CREATE_UNION( union3, 3 ) {
            ...
         }
      }

      // Synchronization of the non-global rigid bodies
      world->synchronize();

      ...

      // Finalizing the MPI system
      // The MPI system must be finalized after the last pe functionality has been used. It
      // is recommended to make MPI_Finalize() the very last call of the main function.
      MPI_Finalize();
   }
   \endcode

 * Note that infinite rigid bodies (such as the Plane primitive) are always considered global
 * rigid bodies. Therefore infinite rigid bodies have to be created inside a global
 * section in case of an MPI parallel simulation.
 */
#define pe_GLOBAL_SECTION \
   if( pe::GlobalSection globalSection = true )
//*************************************************************************************************

} // namespace pe

#endif
