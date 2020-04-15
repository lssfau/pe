//=================================================================================================
/*!
 *  \file src/opendx/Writer.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/math/Vector3.h>
#include <pe/opendx/Writer.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>
#include <pe/util/Time.h>


namespace pe {

namespace opendx {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool Writer::active_( false );
boost::mutex Writer::instanceMutex_;




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor of the Writer class.
 *
 * \param filename File name for the OpenDX visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid file name.
 */
Writer::Writer( const std::string& filename, unsigned int spacing )
   : Visualization()                // Initialization of the Visualization base object
   , Dependency<logging::Logger>()  // Initialization of the logger lifetime dependency
   , spacing_ ( spacing )           // Spacing between two visualized time steps
   , steps_   ( 0 )                 // Time step counter between two time steps
   , counter_ ( 0 )                 // Visualization counter for the number of visualized time steps
   , filename_(filename)            // The filename of the OpenDX file
   , spheres_ ()                    // Registered spheres
{
   // Setting the active flag
   pe_INTERNAL_ASSERT( !active_, "Multiple constructor calls for a singleton class" );
   active_ = true;

   std::ofstream out( filename_.c_str(), std::ofstream::out | std::ofstream::trunc );

   if( !out.is_open() )
      throw std::invalid_argument( "Invalid file name" );

   out << "# OpenDX visualization file\n";
   out << "# " << getTime() << "\n";
   out << "# Author: Klaus Iglberger\n\n";

   out.close();

   // Adding the registered visible spheres
   for( Visualization::Spheres::Iterator s=beginSpheres(); s!=endSpheres(); ++s )
      addSphere( *s );

   // Logging the successful setup of the OpenDX writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the OpenDX writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the Writer class.
 */
Writer::~Writer()
{
   std::ofstream out( filename_.c_str(), std::ofstream::out | std::ofstream::app );
   pe_INTERNAL_ASSERT( out.is_open(), "OpenDX output file could not be opened" );

   for( unsigned int i=1; i<=counter_; ++i )
   {
      out << "object " << counter_+i << " class field\n";
      out << "component \"positions\" value " << i << "\n\n";
   }

   out << "object \"series\" class series\n";

   for( unsigned int i=0; i<counter_; ++i ) {
      out << "member " << i << " value " << counter_+i+1 << " position " << i << "\n";
   }

   out << "\nend\n";

   out.close();

   // Logging the successful destruction of the OpenDX writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the OpenDX writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the spacing of the OpenDX visualization.
 *
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid spacing value.
 */
void Writer::setSpacing( unsigned int spacing )
{
   // Checking the spacing value
   if( spacing_ == 0 )
      throw std::invalid_argument( "Invalid spacing value" );

   spacing_ = spacing;
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single sphere for the OpenDX visualization.
 *
 * \param sphere The sphere to be registered.
 * \return void
 */
void Writer::addSphere( ConstSphereID sphere )
{
   spheres_.pushBack( sphere );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single box for the OpenDX visualization.
 *
 * \param box The box to be registered.
 * \return void
 */
void Writer::addBox( ConstBoxID /*box*/ )
{
   // The Writer is not able to visualize boxes. Therefore the box doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single capsule for the OpenDX visualization.
 *
 * \param capsule The capsule to be registered.
 * \return void
 */
void Writer::addCapsule( ConstCapsuleID /*capsule*/ )
{
   // The Writer is not able to visualize capsules. Therefore the capsule doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single cylinder for the OpenDX visualization.
 *
 * \param cylinder The cylinder to be registered.
 * \return void
 */
void Writer::addCylinder( ConstCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single plane for the OpenDX visualization.
 *
 * \param plane The plane to be registered.
 * \return void
 */
void Writer::addPlane( ConstPlaneID /*plane*/ )
{
   // The Writer is not able to visualize planes. Therefore the plane doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single triangle mesh for the OpenDX visualization.
 *
 * \param mesh The triangle mesh to be registered.
 * \return void
 */
void Writer::addMesh( ConstTriangleMeshID /*mesh*/ )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single spring for the OpenDX visualization.
 *
 * \param spring The spring to be registered.
 * \return void
 */
void Writer::addSpring( ConstSpringID /*spring*/ )
{
   // The Writer is not able to visualize springs. Therefore the spring doesn't
   // have to be registered.
   return;
}
//*************************************************************************************************




//=================================================================================================
//
//  REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Removing a single sphere from the OpenDX visualization.
 *
 * \param sphere The sphere to be removed.
 * \return void
 */
void Writer::removeSphere( ConstSphereID sphere )
{
   for( Spheres::Iterator pos=spheres_.begin(); pos!=spheres_.end(); ++pos ) {
      if( *pos == sphere ) {
         spheres_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the OpenDX visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single box from the OpenDX visualization.
 *
 * \param box The box to be removed.
 * \return void
 */
void Writer::removeBox( ConstBoxID /*box*/ )
{
   // The Writer is not able to visualize boxes. Therefore the box doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single capsule from the OpenDX visualization.
 *
 * \param capsule The capsule to be removed.
 * \return void
 */
void Writer::removeCapsule( ConstCapsuleID /*capsule*/ )
{
   // The Writer is not able to visualize capsules. Therefore the capsule doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single cylinder from the OpenDX visualization.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 */
void Writer::removeCylinder( ConstCylinderID /*cylinder*/ )
{
   // The Writer is not able to visualize cylinders. Therefore the cylinder doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single plane from the OpenDX visualization.
 *
 * \param plane The plane to be removed.
 * \return void
 */
void Writer::removePlane( ConstPlaneID /*plane*/ )
{
   // The Writer is not able to visualize planes. Therefore the planes doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single triangle mesh from the OpenDX visualization.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 */
void Writer::removeMesh( ConstTriangleMeshID /*mesh*/ )
{
   // The Writer is not able to visualize triangle meshes. Therefore the mesh doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single spring from the OpenDX visualization.
 *
 * \param spring The spring to be removed.
 * \return void
 */
void Writer::removeSpring( ConstSpringID /*spring*/ )
{
   // The Writer is not able to visualize springs. Therefore the spring doesn't
   // have to be deregistered.
   return;
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function is automatically called every time step. It extends the OpenDX visualization
 * file by the current state of all registered spheres.
 */
void Writer::trigger()
{
   // Skipping the visualization for intermediate time steps
   if( ++steps_ < spacing_ ) return;

   Vec3 pos;

   // Adjusing the counters
   steps_ = 0;
   ++counter_;

   // Opening the output file
   std::ofstream out( filename_.c_str(), std::ofstream::out | std::ofstream::app );

   if( !out.is_open() )
      return;

   // Writing the registered spheres to file
   out << "object " << counter_ << " class array type float rank 1 shape 3 items " << spheres_.size() << " data follows\n";

   for( Spheres::ConstIterator it=spheres_.begin(); it!=spheres_.end(); ++it )
   {
      // Don't visualize the sphere in case it is invisible
      if( !it->isVisible() ) continue;

      // Writing the sphere position
      pos = it->getPosition();
      if( std::fabs( pos[0] ) < real(1E-20) ) pos[0] = real(0);
      if( std::fabs( pos[1] ) < real(1E-20) ) pos[1] = real(0);
      if( std::fabs( pos[2] ) < real(1E-20) ) pos[2] = real(0);
      out << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
   }

   out << "attribute \"dep\" string \"positions\"\n\n";

   // Closing the output file
   out.close();
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of an OpenDX writer.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Writer::print( std::ostream& os ) const
{
   os << " OpenDX file                  : '" << filename_ << "'\n"
      << " Number of time steps         : "  << counter_  << "\n"
      << " Number of registered spheres : "  << spheres_.size() << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  OPENDX WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Activation of the OpenDX writer.
 * \ingroup opendx
 *
 * \param filename File name for the OpenDX visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \return Handle to the active OpenDX writer.
 * \exception std::invalid_argument Invalid spacing value.
 * \exception std::invalid_argument Invalid file name.
 *
 * This function activates the OpenDX writer for an OpenDX visualization. The first call to
 * this function will activate the OpenDX writer and return the handle to the active writer,
 * subsequent calls will ignore the parameters and only return the handle to the OpenDX writer.
 */
WriterID activateWriter( const std::string& filename, unsigned int spacing=1 )
{
   boost::mutex::scoped_lock lock( Writer::instanceMutex_ );
   static WriterID dx( new Writer( filename, spacing ) );
   return dx;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for OpenDX writers.
 * \ingroup opendx
 *
 * \param os Reference to the output stream.
 * \param dx Reference to a constant OpenDX writer object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Writer& dx )
{
   os << "--" << pe_BROWN << "OPENDX FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for OpenDX writer handles.
 * \ingroup opendx
 *
 * \param os Reference to the output stream.
 * \param dx OpenDX writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const WriterID& dx )
{
   os << "--" << pe_BROWN << "OPENDX FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant OpenDX writer handles.
 * \ingroup opendx
 *
 * \param os Reference to the output stream.
 * \param dx Constant OpenDX writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstWriterID& dx )
{
   os << "--" << pe_BROWN << "OPENDX FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace opendx

} // namespace pe
