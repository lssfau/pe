//=================================================================================================
/*!
 *  \file src/vtk/Writer.cpp
 *  \brief VTK file writer for the VTK visualization
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
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/version.hpp>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/Serialization.h>
#include <pe/core/TimeStep.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>
#include <pe/util/Time.h>
#include <pe/vtk/Writer.h>
#include <pe/vtk/Base64Writer.h>


namespace pe {

namespace vtk {

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
 * \param filename File name for the VTK visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid file name.
 */
Writer::Writer( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary )
   : Visualization()                // Initialization of the Visualization base object
   , Dependency<logging::Logger>()  // Initialization of the logger lifetime dependency
   , tspacing_ ( spacing )           // Spacing between two visualized time steps
   , tstart_ ( tstart )
   , tend_ ( tend )
   , steps_   ( 0 )                 // Time step counter between two time steps
   , counter_ ( 0 )                 // Visualization counter for the number of visualized time steps
   , filename_(filename)            // The output directory for the VTK files
   , binary_(binary)
   , spheres_ ()                    // Registered spheres
   , boxes_ ()						// Registered boxes
   , capsules_ ()					// Registered capsules
   , cylinders_ ()					// Registered cylinders
   , planes_ ()						// Registered planes
   , meshes_ ()						// Registered meshes
   , springs_ ()					// Registered springs
{
   using namespace boost::filesystem;

   // Setting the active flag
   pe_INTERNAL_ASSERT( !active_, "Multiple constructor calls for a singleton class" );
   active_ = true;

   path p( filename_ );

   pe_EXCLUSIVE_SECTION( 0 ) {
      if( exists( p ) ) {
         std::cerr << "vtk::Writer::Writer(): Directory exists: " << p << ". - Files in this directory may be overwritten!\n";
      }
      else {
         create_directory(p);
      }
   }

   if( MPISettings::rank() == MPISettings::root() ) {

      // Write the pvd collector file (Paraview) for all time steps
      std::stringstream pvdfile;
      pvdfile << filename_<<"/collector.pvd";
      std::ofstream pvd( pvdfile.str().c_str(), std::ofstream::out | std::ofstream::trunc );
      if( !pvd.is_open() )
         throw std::invalid_argument( "Invalid file name: " );

      // write header
      pvd << "<?xml version=\"1.0\"?>" << std::endl;
      pvd << "<!-- This pvd file references all the written time steps. -->\n";
      pvd << "<VTKFile type=\"Collection\" version=\"0.1\" byte_order=\"LittleEndian\">" << std::endl;
      pvd << "<Collection>" << std::endl;

      // write filenames of datasets
      int timeCount=1;  // for numbering of the files
      for(unsigned int t=tstart_; t<tend_; t+=tspacing_,timeCount++)
      {
         for(int proc=0; proc<MPISettings::size(); proc++)
         {
            // Write spheres entry
            pvd<< "<DataSet timestep=\"" <<t<<
                  "\" part=\"" << 2*proc <<
                  "\" file=\"" << proc << "/spheres" << timeCount <<".vtu\"/>\n";

            // Write boxes entry
            pvd<< "<DataSet timestep=\"" <<t<<
                  "\" part=\"" << 2*proc+1 <<
                  "\" file=\"" << proc << "/boxes" << timeCount <<".vtu\"/>\n";

            //TODO: Write other entries
         }
      }

      // close tags
      pvd << "</Collection>" << std::endl;
      pvd << "</VTKFile>" << std::endl;
      pvd.close();
   }


   if(MPISettings::rank()==MPISettings::root()) {

      // Write the visit collector file (Visit) for all time steps
      std::stringstream visitfile;
      visitfile << filename_<<"/collector.visit";
      std::ofstream visit( visitfile.str().c_str(), std::ofstream::out | std::ofstream::trunc );
      if( !visit.is_open() )
         throw std::invalid_argument( "Invalid file name: " );

      // write header
      visit << "!NBLOCKS " << 2 * MPISettings::size() << std::endl;

      // write filenames of datasets
      int timeCount=1;  // for numbering of the files
      for(unsigned int t=tstart_; t<tend_; t+=tspacing_,timeCount++)
      {
         for(int proc=0; proc<MPISettings::size(); proc++)
         {
            // Write sphere entry
            visit<< proc << "/spheres" << timeCount <<".vtu\n";

            // Write sphere entry
            visit<< proc << "/boxes" << timeCount <<".vtu\n";

            //TODO: Write other entries
         }
      }

      // close tags
      visit << "</Collection>" << std::endl;
      visit << "</VTKFile>" << std::endl;
      visit.close();
   }



   // Adding the registered visible spheres
   for( Visualization::Spheres::Iterator s=beginSpheres(); s!=endSpheres(); ++s )
      addSphere( *s );

   // Adding the registered visible spheres
   for( Visualization::Boxes::Iterator s=beginBoxes(); s!=endBoxes(); ++s )
      addBox( *s );

   // Logging the successful setup of the VTK writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the VTK writer instance";
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
   pe_INTERNAL_ASSERT( out.is_open(), "VTK output file could not be opened" );

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

   // Logging the successful destruction of the VTK writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the VTK writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the spacing of the VTK visualization.
 *
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid spacing value.
 */
void Writer::setSpacing( unsigned int spacing )
{
   // Checking the spacing value
   if( tspacing_ == 0 )
      throw std::invalid_argument( "Invalid spacing value" );

   tspacing_ = spacing;
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single sphere for the VTK visualization.
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
/*!\brief Registering a single box for the VTK visualization.
 *
 * \param box The box to be registered.
 * \return void
 */
void Writer::addBox( ConstBoxID box )
{
   // The Writer is not able to visualize boxes. Therefore the box doesn't
   // have to be registered.
	boxes_.pushBack( box );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single capsule for the VTK visualization.
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
/*!\brief Registering a single cylinder for the VTK visualization.
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
/*!\brief Registering a single plane for the VTK visualization.
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
/*!\brief Registering a single triangle mesh for the VTK visualization.
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
/*!\brief Registering a single spring for the VTK visualization.
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
/*!\brief Removing a single sphere from the VTK visualization.
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
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the VTK visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single box from the VTK visualization.
 *
 * \param box The box to be removed.
 * \return void
 */
void Writer::removeBox( ConstBoxID box )
{
	   for( Boxes::Iterator pos=boxes_.begin(); pos!=boxes_.end(); ++pos ) {
	      if( *pos == box ) {
	         boxes_.erase( pos );
	         return;
	      }
	   }
	   pe_INTERNAL_ASSERT( false, "Box is not registered for the VTK visualization" );
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single capsule from the VTK visualization.
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
/*!\brief Removing a single cylinder from the VTK visualization.
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
/*!\brief Removing a single plane from the VTK visualization.
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
/*!\brief Removing a single triangle mesh from the VTK visualization.
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
/*!\brief Removing a single spring from the VTK visualization.
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
/*!\brief Visualizing the current state of the registered rigid bodies.
 *
 * \return void
 *
 * This function is automatically called every time step. It extends the VTK visualization
 * file by the current state of all registered spheres.
 */
void Writer::trigger()
{
   // Skipping the visualization for intermediate time steps
   if( ++steps_ < tspacing_ ) return;

   // Adjusing the counters
   steps_ = 0;
   ++counter_;

   std::ostringstream spherefile;
   spherefile << filename_ << "/spheres" << counter_ << ".vtu";
   writeSpheres( spherefile.str().c_str() );

   std::ostringstream boxfile;
   boxfile << filename_ << "/boxes" << counter_ << ".vtu";
   writeBoxes( boxfile.str().c_str() );
}
//*************************************************************************************************




//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeSpheres(const boost::filesystem::path& filename) const
{
	   using namespace boost::filesystem;
	   using boost::lexical_cast;

	   // Checking if the function is called inside an exclusive section
	   if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
	      throw std::runtime_error( "Invalid function call inside exclusive section" );
	   }

	   // Determining the directory and the filename for the POV-Ray visualization
	   const path directory( filename.parent_path() );
	   const path file     ( filename.filename()    );

	   // Checking the directory and the filename
	   if( !directory.empty() && !exists( directory ) )
	      throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
	   if( file.empty() )
	      throw std::runtime_error( "Invalid file name" );

	   // Generation of a single VTK-Ray file
	   // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
	   // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
	   // ascending order.
	   if( false /*singleFile_  || MPISettings::size() == 1 */)
	   {
	      pe_SERIALIZATION
	      {
	    	  throw std::runtime_error( "src/vtk/Writer.cpp::writeSpheres(): not yet implemented");

	         // Opening the output file
	         std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
	         if( !out.is_open() ) {
	            std::ostringstream oss;
	            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
	            throw std::runtime_error( oss.str() );
	         }

	         // Writing the registered spheres
	         for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
	         {

	         }

	         // Closing the output file
	         out.close();
	      }
	   }

	   // Generation of multiple VTK-Ray files
	   // In case the 'singleFile' flag is set to 'false' each process creates an individual
	   // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
	   // file 'filename'.
	   else
	   {
	      // Creating the process-specific extended directory
	      path extended( directory );
	      extended /= lexical_cast<std::string>( MPISettings::rank() );
	      if( !exists( extended ) )
	         create_directory( extended );
	      extended /= file;

	      // Opening the output file
	      std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
	      if( !out.is_open() ) {
	         std::ostringstream oss;
	         oss << " Error opening VTK-Ray file '" << filename << "' !\n";
	         throw std::runtime_error( oss.str() );
	      }

         if(binary_)
         {
            //std::cout << "writing spheres binary";
            writeSphereDataBinary(out);
         }
         else
         {
            //std::cout << "writing spheres ascii";
            writeSphereDataAscii(out);
         }

	      // Closing the output file
	      out.close();
	   }
}
//*************************************************************************************************



void Writer::writeSphereDataBinary(std::ostream& out) const {
   Base64Writer b64(out);

   unsigned int numSpheres = boost::numeric_cast<unsigned int>( spheres_.size() );
   unsigned int data_size_tensor = (4 * sizeof(float) * (numSpheres * 9)
         + 2) / 3;
   unsigned int data_size_vec = (4 * sizeof(float) * (numSpheres * 3) + 2)
         / 3;
   unsigned int data_size_scalar = (4 * sizeof(float) * (numSpheres * 1)
         + 2) / 3;

   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out
         << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << spheres_.size()
         << "\" NumberOfCells=\"" << spheres_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" << "\" NumberOfComponents=\""
         << 3 << "\" format=\"binary\">\n";

   // write array size first
   b64 << data_size_vec;
   b64.flush();

   // Write the sphere positions
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getPosition()[0];
      b64 << (float)s->getPosition()[1];
      b64 << (float)s->getPosition()[2];
   }
   b64.flush();

   // Declare grid as point cloud
   out << "    </DataArray>\n";
   out << "   </Points>\n";
   out << "   <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << i << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << i + 1 << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for (unsigned int i = 0; i < spheres_.size(); i++)
      out << " " << 1 << "\n";
   out << "    </DataArray>\n";
   out << "   </Cells>\n";

   // Data at each point
   out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

   // write IDs
   out << "    <DataArray type=\"" << "UInt32" << "\" Name=\"" << "ID"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << s->getID();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Radii
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Radius"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getRadius();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Mass
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Mass"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      b64 << (float)s->getMass();
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Orientation
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Orientation" << "\" NumberOfComponents=\"" << 9
         << "\" format=\"binary\">\n";
   b64 << data_size_tensor;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const pe::Rot3& rot = s->getRotation();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      b64 << (float)rot[3];
      b64 << (float)rot[4];
      b64 << (float)rot[5];
      b64 << (float)rot[6];
      b64 << (float)rot[7];
      b64 << (float)rot[8];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Euler angles
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Euler Rotation" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Net Force
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Net Force"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &f = s->getForce();
      b64 << (float)f[0];
      b64 << (float)f[1];
      b64 << (float)f[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write Velocsy
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Velocity"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &v = s->getLinearVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   // write angular Velocity
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Angular Velocity" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Spheres::ConstIterator s = spheres_.begin(); s != spheres_.end(); ++s) {
      const Vec3 &v = s->getAngularVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Spheres
   b64.flush();
   out << "    </DataArray>\n";

   out << "   </PointData>\n";
   out << "   <CellData>  </CellData>\n";
   out << "  </Piece>\n";
   out << " </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}





void Writer::writeSphereDataAscii(std::ostream& out) const
{
   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << spheres_.size() <<
          "\" NumberOfCells=\"" << spheres_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
          "\" NumberOfComponents=\"" << 3 <<
          "\" format=\"ascii\">\n";

     // Write the sphere positions
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
       out << "\t" << s->getPosition()[0] << "\t" << s->getPosition()[1] << "\t" << s->getPosition()[2] << "\n";
     }

     // Declare grid as point cloud
     out << "    </DataArray>\n";
     out << "   </Points>\n";
     out << "   <Cells>\n";
     out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << i << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << i + 1 << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
     for (unsigned int i = 0; i < spheres_.size(); i++)
        out << " " << 1 << "\n";
     out << "    </DataArray>\n";
     out << "   </Cells>\n";

     // Data at each point
     out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

     // write IDs
     out << "    <DataArray type=\"" << "UInt32" <<
            "\" Name=\"" << "ID" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getID() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Radii
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Radius" <<
           "\" NumberOfComponents=\"" << 1 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getRadius() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Mass
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Mass" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        out << "\t" << s->getMass() << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Orientation
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Orientation" <<
            "\" NumberOfComponents=\"" << 9 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const pe::Rot3& rot = s->getRotation();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << " " << rot[1] << " " << rot[2] <<
               "\t" << rot[3] << " " << rot[4] << " " << rot[5] <<
               "\t" << rot[6] << " " << rot[7] << " " << rot[8] <<"\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Euler angles
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Euler Rotation" <<
            "\" NumberOfComponents=\"" << 3 <<
            "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << "\t" << rot[1] << "\t" << rot[2] << "\n";
        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Net Force
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Net Force" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &f = s->getForce();
        out << "\t" << f[0] << "\t" << f[1] << "\t" << f[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write Velocsy
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &v = s->getLinearVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     // write angular Velocity
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Angular Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
     {
        const Vec3 &v = s->getAngularVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Spheres
     out << "    </DataArray>\n";

     out << "   </PointData>\n";
     out << "   <CellData>  </CellData>\n";
     out << "  </Piece>\n";
     out << " </UnstructuredGrid>\n";
     out << "</VTKFile>\n";
}




//*************************************************************************************************
/*!\brief Visualizing the current state of the registered spheres.
 *
 * \return void
 *
 * This function creates a new file containing the sphere data of the current timestep
 */
void Writer::writeBoxes(const boost::filesystem::path& filename) const
{
      using namespace boost::filesystem;
      using boost::lexical_cast;

      // Checking if the function is called inside an exclusive section
      if( MPISettings::size() > 1 && ExclusiveSection::isActive() ) {
         throw std::runtime_error( "Invalid function call inside exclusive section" );
      }

      // Determining the directory and the filename for the POV-Ray visualization
      const path directory( filename.parent_path() );
      const path file     ( filename.filename()    );

      // Checking the directory and the filename
      if( !directory.empty() && !exists( directory ) )
         throw std::runtime_error( "Directory for VTK-Ray files does not exist" );
      if( file.empty() )
         throw std::runtime_error( "Invalid file name" );

      // Generation of a single VTK-Ray file
      // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
      // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
      // ascending order.
      if( false && /*singleFile || */ MPISettings::size() == 1 )
      {
         pe_SERIALIZATION
         {
           throw std::runtime_error( "src/vtk/Writer.cpp::writeSpheres(): not yet implemented");

            // Opening the output file
            std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
            if( !out.is_open() ) {
               std::ostringstream oss;
               oss << " Error opening VTK-Ray file '" << filename << "' !\n";
               throw std::runtime_error( oss.str() );
            }

            // Writing the registered boxes
            for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
            {

            }

            // Closing the output file
            out.close();
         }
      }

      // Generation of multiple VTK-Ray files
      // In case the 'singleFile' flag is set to 'false' each process creates an individual
      // POV-Ray file (containing their local rigid bodies) that is included in the main POV-Ray
      // file 'filename'.
      else
      {
         // Creating the process-specific extended directory
         path extended( directory );
         extended /= lexical_cast<std::string>( MPISettings::rank() );
         if( !exists( extended ) )
            create_directory( extended );
         extended /= file;

         // Opening the output file
         std::ofstream out( extended.string().c_str(), std::ofstream::out | std::ostream::trunc );
         if( !out.is_open() ) {
            std::ostringstream oss;
            oss << " Error opening VTK-Ray file '" << filename << "' !\n";
            throw std::runtime_error( oss.str() );
         }

         if(binary_) {
            //std::cout << "writing boxes binary";
            writeBoxDataBinary(out);
         }
         else {
            //std::cout << "writing boxes ascii";
            writeBoxDataAscii(out);
         }

         // Closing the output file
         out.close();
      }
}
//*************************************************************************************************



void Writer::writeBoxDataAscii(std::ostream& out) const {
   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << boxes_.size() <<
        "\" NumberOfCells=\"" << boxes_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" <<
        "\" NumberOfComponents=\"" << 3 <<
        "\" format=\"ascii\">\n";

     // Write the sphere positions
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
       out << "\t" << s->getPosition()[0] << "\t" << s->getPosition()[1] << "\t" << s->getPosition()[2] << "\n";
     }

     // Declare grid as point cloud
     out << "    </DataArray>\n";
     out << "   </Points>\n";
     out << "   <Cells>\n";
     out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
     for (unsigned int i = 0; i < boxes_.size(); i++)
        out << " " << i << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
     for (unsigned int i = 0; i < boxes_.size(); i++)
        out << " " << i + 1 << "\n";
     out << "    </DataArray>\n";
     out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
     for (unsigned int i = 0; i < boxes_.size(); i++)
        out << " " << 1 << "\n";
     out << "    </DataArray>\n";
     out << "   </Cells>\n";

     // Data at each point
     out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

     // write IDs
     out << "    <DataArray type=\"" << "UInt32" <<
            "\" Name=\"" << "ID" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        out << "\t" << s->getID() << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Radii
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Lengths" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        out << "\t" << s->getLengths()[0] << "\t" << s->getLengths()[1] << "\t" << s->getLengths()[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Mass
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Mass" <<
            "\" NumberOfComponents=\"" << 1 <<
            "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        out << "\t" << s->getMass() << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Orientation
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Orientation" <<
            "\" NumberOfComponents=\"" << 9 <<
            "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        const pe::Rot3& rot = s->getRotation();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << " " << rot[1] << " " << rot[2] <<
               "\t" << rot[3] << " " << rot[4] << " " << rot[5] <<
               "\t" << rot[6] << " " << rot[7] << " " << rot[8] <<"\n";
        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Euler angles
     out << "    <DataArray type=\"" << "Float32" <<
            "\" Name=\"" << "Euler Rotation" <<
            "\" NumberOfComponents=\"" << 3 <<
            "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
        //Vector3<Real> o = calcEulerAngles(rot);
        out << "\t" << rot[0] << "\t" << rot[1] << "\t" << rot[2] << "\n";
        //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Net Force
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Net Force" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        const Vec3 &f = s->getForce();
        out << "\t" << f[0] << "\t" << f[1] << "\t" << f[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write Velocsy
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        const Vec3 &v = s->getLinearVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     // write angular Velocity
     out << "    <DataArray type=\"" << "Float32" <<
           "\" Name=\"" << "Angular Velocity" <<
           "\" NumberOfComponents=\"" << 3 <<
           "\" format=\"ascii\">\n";
     for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
     {
        const Vec3 &v = s->getAngularVel();
        out << "\t" << v[0] << "\t" << v[1] << "\t" << v[2] << "\n";
     }      //end for all Boxes
     out << "    </DataArray>\n";

     out << "   </PointData>\n";
     out << "   <CellData>  </CellData>\n";
     out << "  </Piece>\n";
     out << " </UnstructuredGrid>\n";
     out << "</VTKFile>\n";
}



void Writer::writeBoxDataBinary(std::ostream& out) const {
   Base64Writer b64(out);

   unsigned int numBoxes = boost::numeric_cast<unsigned int>( boxes_.size() );
   unsigned int data_size_tensor = (4 * sizeof(float) * (numBoxes * 9) + 2) / 3;
   unsigned int data_size_vec = (4 * sizeof(float) * (numBoxes * 3) + 2) / 3;
   unsigned int data_size_scalar = (4 * sizeof(float) * (numBoxes * 1) + 2) / 3;

   // write grid
   out << "<?xml version=\"1.0\"?>\n";
   out << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
   out << " <UnstructuredGrid>\n";
   out << "  <Piece NumberOfPoints=\"" << boxes_.size()
         << "\" NumberOfCells=\"" << boxes_.size() << "\">\n";
   out << "   <Points>\n";
   out << "    <DataArray type=\"" << "Float32" << "\" NumberOfComponents=\""
         << 3 << "\" format=\"binary\">\n";

//
//   // Write the sphere positions
//   for( Boxes::ConstIterator s=boxes_.begin(); s!=boxes_.end(); ++s )
//   {
//     out << "\t" << s->getPosition()[0] << "\t" << s->getPosition()[1] << "\t" << s->getPosition()[2] << "\n";
//   }


   // write array size first
   b64 << data_size_vec;
   b64.flush();

   // Write the sphere positions
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      b64 << (float)s->getPosition()[0];
      b64 << (float)s->getPosition()[1];
      b64 << (float)s->getPosition()[2];
   }
   b64.flush();

   // Declare grid as point cloud
   out << "    </DataArray>\n";
   out << "   </Points>\n";
   out << "   <Cells>\n";
   out << "    <DataArray type=\"Int32\" Name=\"connectivity\">\n";
   for (unsigned int i = 0; i < boxes_.size(); i++)
      out << " " << i << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"Int32\" Name=\"offsets\">\n";
   for (unsigned int i = 0; i < boxes_.size(); i++)
      out << " " << i + 1 << "\n";
   out << "    </DataArray>\n";
   out << "    <DataArray type=\"UInt8\" Name=\"types\">\n";
   for (unsigned int i = 0; i < boxes_.size(); i++)
      out << " " << 1 << "\n";
   out << "    </DataArray>\n";
   out << "   </Cells>\n";

   // Data at each point
   out << "   <PointData Scalars=\"scalars\" Vectors=\"vectors\">\n";

   // write IDs
   out << "    <DataArray type=\"" << "UInt32" << "\" Name=\"" << "ID"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      b64 << s->getID();
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Radii
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Lengths"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      b64 << (float)s->getLengths()[0];
      b64 << (float)s->getLengths()[1];
      b64 << (float)s->getLengths()[2];
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Mass
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Mass"
         << "\" NumberOfComponents=\"" << 1 << "\" format=\"binary\">\n";
   b64 << data_size_scalar;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      b64 << (float)s->getMass();
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Orientation
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Orientation" << "\" NumberOfComponents=\"" << 9
         << "\" format=\"binary\">\n";
   b64 << data_size_tensor;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      const pe::Rot3& rot = s->getRotation();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      b64 << (float)rot[3];
      b64 << (float)rot[4];
      b64 << (float)rot[5];
      b64 << (float)rot[6];
      b64 << (float)rot[7];
      b64 << (float)rot[8];
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Euler angles
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Euler Rotation" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      const pe::Vec3& rot = s->getRotation().getEulerAnglesXYZ();
      //Vector3<Real> o = calcEulerAngles(rot);
      b64 << (float)rot[0];
      b64 << (float)rot[1];
      b64 << (float)rot[2];
      //out << "\t" << o[0] << "\t" << o[1] << "\t" << o[2] << "\n";
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Net Force
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Net Force"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      const Vec3 &f = s->getForce();
      b64 << (float)f[0];
      b64 << (float)f[1];
      b64 << (float)f[2];
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write Velocsy
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\"" << "Velocity"
         << "\" NumberOfComponents=\"" << 3 << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      const Vec3 &v = s->getLinearVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   // write angular Velocity
   out << "    <DataArray type=\"" << "Float32" << "\" Name=\""
         << "Angular Velocity" << "\" NumberOfComponents=\"" << 3
         << "\" format=\"binary\">\n";
   b64 << data_size_vec;
   b64.flush();
   for (Boxes::ConstIterator s = boxes_.begin(); s != boxes_.end(); ++s) {
      const Vec3 &v = s->getAngularVel();
      b64 << (float)v[0];
      b64 << (float)v[1];
      b64 << (float)v[2];
   } //end for all Boxes
   b64.flush();
   out << "    </DataArray>\n";

   out << "   </PointData>\n";
   out << "   <CellData>  </CellData>\n";
   out << "  </Piece>\n";
   out << " </UnstructuredGrid>\n";
   out << "</VTKFile>\n";
}





//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of an VTK writer.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Writer::print( std::ostream& os ) const
{
   os << " VTK file                     : '" << filename_ << "'\n"
      << " Number of time steps         : "  << counter_  << "\n"
      << " Number of registered spheres : "  << spheres_.size() << "\n"
      << " Number of registered boxes   : "  << boxes_.size() << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  VTK WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Activation of the VTK writer.
 * \ingroup vtk
 *
 * \param filename File name for the VTK visualization file.
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \return Handle to the active VTK writer.
 * \exception std::invalid_argument Invalid spacing value.
 * \exception std::invalid_argument Invalid file name.
 *
 * This function activates the VTK writer for an VTK visualization. The first call to
 * this function will activate the VTK writer and return the handle to the active writer,
 * subsequent calls will ignore the parameters and only return the handle to the VTK writer.
 */
WriterID activateWriter( const std::string& filename, unsigned int spacing, unsigned int tstart, unsigned int tend, bool binary=false )
{
   boost::mutex::scoped_lock lock( Writer::instanceMutex_ );
   static WriterID dx( new Writer( filename, spacing, tstart, tend, binary ) );
   return dx;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for VTK writers.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx Reference to a constant VTK writer object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Writer& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for VTK writer handles.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx VTK writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const WriterID& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant VTK writer handles.
 * \ingroup vtk
 *
 * \param os Reference to the output stream.
 * \param dx Constant VTK writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstWriterID& dx )
{
   os << "--" << pe_BROWN << "VTK FILE WRITER" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   dx->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace vtk

} // namespace pe
