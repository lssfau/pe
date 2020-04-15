//=================================================================================================
/*!
 *  \file pe/opendx/Writer.h
 *  \brief OpenDX file writer for the OpenDX visualization
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

#ifndef _PE_OPENDX_WRITER_H_
#define _PE_OPENDX_WRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <string>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <pe/core/Types.h>
#include <pe/core/Visualization.h>
#include <pe/opendx/WriterID.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/singleton/Dependency.h>


namespace pe {

namespace opendx {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief OpenDX visualization writer.
 * \ingroup opendx
 *
 * The Writer class offers the functionality to create OpenDX files for a visualization of the
 * rigid body simulation. However, the Writer is only able to visualize spheres.\n
 * In order to activate the OpenDX visualization use the following function:

   \code
   pe::opendx::WriterID pe::opendx::activateWriter( const std::string& filename, unsigned int spacing );
   \endcode

 * This function activates the OpenDX writer and returns the handle to the active writer.
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
   explicit Writer( const std::string& filename, unsigned int spacing );
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
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   unsigned int spacing_;  //!< Spacing between two visualized time steps.
   unsigned int steps_;    //!< Time step counter between two time steps.
   unsigned int counter_;  //!< Visualization counter for the number of visualized time steps.
   std::string filename_;  //!< The filename of the OpenDX visualization file.
   Spheres spheres_;       //!< The registered spheres for the visualization.

   static bool         active_;         //!< Active flag of the OpenDX writer.
   static boost::mutex instanceMutex_;  //!< Synchronization mutex for access to the OpenDX writer.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend bool     isActive();
   friend WriterID activateWriter( const std::string& filename, unsigned int spacing );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  OPENDX WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name OpenDX writer setup functions */
//@{
inline bool     isActive();
       WriterID activateWriter( const std::string& filename, unsigned int spacing );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the OpenDX visualization is active or not.
 * \ingroup opendx
 *
 * \return \a true if the OpenDX visualization is active, \a false if not.
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
/*!\name OpenDX writer operators */
//@{
std::ostream& operator<<( std::ostream& os, const Writer& dx );
std::ostream& operator<<( std::ostream& os, const WriterID& dx );
std::ostream& operator<<( std::ostream& os, const ConstWriterID& dx );
//@}
//*************************************************************************************************

} // namespace opendx

} // namespace pe

#endif
