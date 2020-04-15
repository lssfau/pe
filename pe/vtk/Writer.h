//=================================================================================================
/*!
 *  \file pe/vtk/Writer.h
 *  \brief Header of the VTK file writer for the VTK visualization
 *
 *  Copyright (C) 2012 Simon Bogner
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

#ifndef _PE_VTK_WRITER_H_
#define _PE_VTK_WRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <pe/core/Types.h>
#include <pe/core/Visualization.h>
#include <pe/vtk/WriterID.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/singleton/Dependency.h>



namespace pe {

namespace vtk {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief VTK visualization writer.
 * \ingroup vtk
 *
 * The Writer class offers the functionality to create VTK files for a visualization of the
 * rigid body simulation. However, the Writer only writes the particle data, it does not
 * take care of full visualization. This is left to the user.\n
 * In order to activate the VTK visualization use the following function:

   \code
   pe::vtk::WriterID pe::vtk::activateWriter( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary );
   \endcode

 * This function activates the VTK writer and returns the handle to the active writer.
 * Subsequent calls of this function will only return the handle to the writer.
 */
class Writer : public Visualization
             , private Dependency<logging::Logger>
{
private:
   //**Type definitions****************************************************************************
   typedef PtrVector<const Sphere,NoDelete>  Spheres;  //!< Vector for registered sphere primitives.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Writer( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary=true);
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Writer();
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setSpacing( unsigned int spacing );
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

private:
   //**Add functions*******************************************************************************
   /*!\name Add functions */
   //@{
   virtual void addSphere  ( ConstSphereID       sphere   );
   virtual void addBox     ( ConstBoxID          box      );
   virtual void addCapsule ( ConstCapsuleID      capsule  );
   virtual void addCylinder( ConstCylinderID     cylinder );
   virtual void addPlane   ( ConstPlaneID        plane    );
   virtual void addMesh    ( ConstTriangleMeshID mesh     );
   virtual void addSpring  ( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Remove functions****************************************************************************
   /*!\name Remove functions */
   //@{
   virtual void removeSphere  ( ConstSphereID       sphere   );
   virtual void removeBox     ( ConstBoxID          box      );
   virtual void removeCapsule ( ConstCapsuleID      capsule  );
   virtual void removeCylinder( ConstCylinderID     cylinder );
   virtual void removePlane   ( ConstPlaneID        plane    );
   virtual void removeMesh    ( ConstTriangleMeshID mesh     );
   virtual void removeSpring  ( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Visualization functions*********************************************************************
   /*!\name Visualization functions */
   //@{
   virtual void trigger();

   virtual void writeSpheres(const boost::filesystem::path& filename) const;
   virtual void writeBoxes(const boost::filesystem::path& filename) const;

   virtual void writeSphereDataAscii(std::ostream& out) const;
   virtual void writeSphereDataBinary(std::ostream& out) const;

   virtual void writeBoxDataAscii(std::ostream& out) const;
   virtual void writeBoxDataBinary(std::ostream& out) const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   unsigned int tspacing_;       //!< Spacing between two visualized time steps.
   unsigned int tstart_;         //!< First time step to be written
   unsigned int tend_;           //!< last time step to be written
   unsigned int steps_;          //!< Time step counter between two time steps.
   unsigned int counter_;        //!< Visualization counter for the number of visualized time steps.
   std::string filename_;        //!< The name of the VTK visualization output directory.
   std::string prefix_;         //!< Prefix of the file name for the POV-Ray files.
   std::string postfix_;        //!< Postfix of the file name for the POV-Ray files.
   bool binary_;                //!< write binary if true; write ascii else
   //bool singleFile_;            //!< write all data into a single file if true;  else

   Spheres spheres_;            //!< Registered spheres for the visualization.
   Boxes boxes_;                //!< Registered boxes for the visualization.
   Capsules capsules_;          //!< Registered capsules for the visualization.
   Cylinders cylinders_;        //!< Registered capsules for the visualization.
   Planes planes_;              //!< Registered planes for the visualization.
   Meshes meshes_;              //!< Registered triangle meshes for the visualization.
   Springs springs_;            //!< Registered springs for the visualization.

   static bool         active_;         //!< Active flag of the VTK writer.
   static boost::mutex instanceMutex_;  //!< Synchronization mutex for access to the VTK writer.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend bool     isActive();
   friend WriterID activateWriter( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary);
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  VTK WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name VTK writer setup functions */
//@{
inline bool     isActive();
       WriterID activateWriter( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary);
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the VTK visualization is active or not.
 * \ingroup vtk
 *
 * \return \a true if the VTK visualization is active, \a false if not.
 */
inline bool isActive()
{
   return Writer::active_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name VTK writer operators */
//@{
std::ostream& operator<<( std::ostream& os, const Writer& dx );
std::ostream& operator<<( std::ostream& os, const WriterID& dx );
std::ostream& operator<<( std::ostream& os, const ConstWriterID& dx );
//@}
//*************************************************************************************************

} // namespace vtk

} // namespace pe


#endif /* WRITER_H_ */
