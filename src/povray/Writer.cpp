//=================================================================================================
/*!
 *  \file src/povray/Writer.cpp
 *  \brief POV-Ray file writer for the POV-Ray visualization
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/version.hpp>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/Serialization.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/TimeStep.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/math/Infinity.h>
#include <pe/povray/ColorPigment.h>
#include <pe/povray/DefaultTexture.h>
#include <pe/povray/PlainTexture.h>
#include <pe/povray/Writer.h>
#include <pe/system/POVRay.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>
#include <pe/util/Time.h>

#include <pe/core/rigidbody/TriangleMeshTypes.h>

namespace pe {

namespace povray {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool Writer::active_( false );
boost::mutex Writer::instanceMutex_;




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The constructor of the Writer class.
 */
Writer::Writer()
   : Visualization()                // Initialization of the Visualization base object
   , Dependency<logging::Logger>()  // Initialization of the logger lifetime dependency
   , Dependency<Camera>()           // Initialization of the camera lifetime dependency
   , start_(0)                      // First visualized time step
   , end_(inf)                      // Last visualized time step
   , spacing_(1)                    // Spacing between two visualized time steps
   , steps_(0)                      // Time step counter between two time steps
   , counter_(0)                    // Internal file counter
   , prefix_("./image")             // Prefix of the file name
   , postfix_(".pov")               // Postfix of the file name
   , background_()                  // Background color
   , policy_()                      // The active texture policy
   , includes_()                    // Included POV-Ray header files
   , declarations_()                // Declared POV-Ray identifiers
   , lightsources_()                // Registered light sources
   , decorations_(true)             // Output camera and light sources
   , spheres_()                     // Registered spheres
   , boxes_()                       // Registered boxes
   , capsules_()                    // Registered capsules
   , cylinders_()                   // Registered cylinders
   , planes_()                      // Registered planes
   , meshes_()                      // Registered triangle meshes
{
   // Setting the active flag
   pe_INTERNAL_ASSERT( !active_, "Multiple constructor calls for a singleton class" );
   active_ = true;

   // Setup of the default textures for finite and infinite rigid bodies
   policy_.reset( new DefaultTexture( PlainTexture( ColorPigment( static_cast<real>(1.0), static_cast<real>(0.6), static_cast<real>(0.0) ) ),
                                      PlainTexture( ColorPigment( static_cast<real>(0.5), static_cast<real>(0.5), static_cast<real>(0.5) ) ) ) );

   // Adding the registered visible spheres
   for( Visualization::Spheres::Iterator s=beginSpheres(); s!=endSpheres(); ++s )
      addSphere( *s );

   // Adding the registered visible boxes
   for( Visualization::Boxes::Iterator b=beginBoxes(); b!=endBoxes(); ++b )
      addBox( *b );

   // Adding the registered visible capsules
   for( Visualization::Capsules::Iterator c=beginCapsules(); c!=endCapsules(); ++c )
      addCapsule( *c );

   // Adding the registered visible cylinders
   for( Visualization::Cylinders::Iterator c=beginCylinders(); c!=endCylinders(); ++c )
      addCylinder( *c );

   // Adding the registered visible planes
   for( Visualization::Planes::Iterator p=beginPlanes(); p!=endPlanes(); ++p )
      addPlane( *p );

   // Adding the registered visible triangle meshes
   for( Visualization::Meshes::Iterator m=beginMeshes(); m!=endMeshes(); ++m )
      addMesh( *m );

   // Adding the registered visible springs
   for( Visualization::Springs::Iterator s=beginSprings(); s!=endSprings(); ++s )
      addSpring( *s );

   // Logging the successful setup of the POV-Ray writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the POV-Ray writer instance";
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
   // Logging the successful destruction of the POV-Ray writer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the POV-Ray writer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the file name of the POV-Ray writer.
 *
 * \param filename File name template for the POV-Ray visualization files.
 * \return void
 * \exception std::invalid_argument File name exceeds 100 characters.
 * \exception std::invalid_argument Invalid file name template.
 *
 * The given file name specifies both the path and the file names of the POV-Ray files generated
 * with the \a Writer::writeFile() function. Since an arbitrary number of files could be created,
 * the given file name is used as a template to determine the file names. The given string may
 * not exceed 100 charecters and has to contain exactly one '\%' sign, which will be replaced by
 * the current number of the POV-Ray image. For instance, the file name "image%.pov" is used to
 * create the POV-Ray images "image0.pov", "image1.pov", ... The default file name that is used
 * if no other valid file name is specified is "image%.pov".
 */
void Writer::setFilename( const boost::filesystem::path& filename )
{
   using namespace boost::filesystem;

   const std::string total ( filename.string()            );
#if (BOOST_VERSION / 10000) >= 1 && (BOOST_VERSION / 100 % 1000) >= 46
   const std::string file  ( filename.filename().string() );
#else
   const std::string file  ( filename.filename()          );
#endif
   const path        parent( filename.parent_path()       );

   // Checking the given directory and filename
   if( !parent.empty() && !exists( parent ) )
      throw std::invalid_argument( "Directory for POV-Ray files does not exist" );
   if( file.empty() )
      throw std::invalid_argument( "Invalid file name" );

   // Limiting the length of the filename to 100 characters
   if( file.size() > 100 )
      throw std::invalid_argument( "File name exceeds 100 characters" );

   std::string::size_type pos( std::string::npos );

   // Checking for a valid file template
   if( ( pos = file.find_first_of( '%', 0 ) ) == std::string::npos ||
       file.find_last_of( '%', std::string::npos ) != pos )
      throw std::invalid_argument( "Invalid file name template" );

   // Creating the pre- and postfix
   pos      = total.find_last_of( '%', std::string::npos );
   prefix_  = total.substr( 0, pos );
   postfix_ = total.substr( pos+1  );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the number of the first visualized time step.
 *
 * \param start Number of the first visualized time step.
 */
void Writer::setStart( size_t start )
{
   start_ = start;
   if( TimeStep::step() > start_ )
      steps_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the number of steps skipped since the last output.
 *
 * \param steps The number of steps skipped since the last output.
 *
 * Note that this function should only be used to restore the PovRay Writer state on program
 * restart.
 */
void Writer::setSteps( size_t steps )
{
   steps_ = steps;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the counting number of the next file output.
 *
 * \param counter The counting number of the next file output.
 *
 * Note that this function should only be used to restore the PovRay Writer state on program
 * restart.
 */
void Writer::setFileCounter( size_t counter )
{
   counter_ = counter;
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single sphere for the POV-Ray visualization.
 *
 * \param sphere The sphere to be registered.
 * \return void
 */
void Writer::addSphere( ConstSphereID sphere )
{
   spheres_.pushBack( SphereData( sphere, policy_->getTexture( sphere ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single box for the POV-Ray visualization.
 *
 * \param box The box to be registered.
 * \return void
 */
void Writer::addBox( ConstBoxID box )
{
   boxes_.pushBack( BoxData( box, policy_->getTexture( box ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single capsule for the POV-Ray visualization.
 *
 * \param capsule The capsule to be registered.
 * \return void
 */
void Writer::addCapsule( ConstCapsuleID capsule )
{
   capsules_.pushBack( CapsuleData( capsule, policy_->getTexture( capsule ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single cylinder for the POV-Ray visualization.
 *
 * \param cylinder The cylinder to be registered.
 * \return void
 */
void Writer::addCylinder( ConstCylinderID cylinder )
{
   cylinders_.pushBack( CylinderData( cylinder, policy_->getTexture( cylinder ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single plane for the POV-Ray visualization.
 *
 * \param plane The plane to be registered.
 * \return void
 */
void Writer::addPlane( ConstPlaneID plane )
{
   planes_.pushBack( PlaneData( plane, policy_->getTexture( plane ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single triangle mesh for the POV-Ray visualization.
 *
 * \param mesh The triangle mesh to be registered.
 * \return void
 */
void Writer::addMesh( ConstTriangleMeshID mesh )
{
   meshes_.pushBack( MeshData( mesh, policy_->getTexture( mesh ) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single spring for the POV-Ray visualization.
 *
 * \param spring The spring to be registered.
 * \return void
 */
void Writer::addSpring( ConstSpringID spring )
{
   springs_.pushBack( SpringData( spring ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Removing a single sphere from the POV-Ray visualization.
 *
 * \param sphere The sphere to be removed.
 * \return void
 */
void Writer::removeSphere( ConstSphereID sphere )
{
   for( Spheres::Iterator pos=spheres_.begin(); pos!=spheres_.end(); ++pos ) {
      if( pos->sphere_ == sphere ) {
         spheres_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single box from the POV-Ray visualization.
 *
 * \param box The box to be removed.
 * \return void
 */
void Writer::removeBox( ConstBoxID box )
{
   for( Boxes::Iterator pos=boxes_.begin(); pos!=boxes_.end(); ++pos ) {
      if( pos->box_ == box ) {
         boxes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Box is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single capsule from the POV-Ray visualization.
 *
 * \param capsule The capsule to be removed.
 * \return void
 */
void Writer::removeCapsule( ConstCapsuleID capsule )
{
   for( Capsules::Iterator pos=capsules_.begin(); pos!=capsules_.end(); ++pos ) {
      if( pos->capsule_ == capsule ) {
         capsules_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Capsule is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single cylinder from the POV-Ray visualization.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 */
void Writer::removeCylinder( ConstCylinderID cylinder )
{
   for( Cylinders::Iterator pos=cylinders_.begin(); pos!=cylinders_.end(); ++pos ) {
      if( pos->cylinder_ == cylinder ) {
         cylinders_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Cylinder is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single plane from the POV-Ray visualization.
 *
 * \param plane The plane to be removed.
 * \return void
 */
void Writer::removePlane( ConstPlaneID plane )
{
   for( Planes::Iterator pos=planes_.begin(); pos!=planes_.end(); ++pos ) {
      if( pos->plane_ == plane ) {
         planes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Plane is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single triangle mesh from the POV-Ray visualization.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 */
void Writer::removeMesh( ConstTriangleMeshID mesh )
{
   for( Meshes::Iterator pos=meshes_.begin(); pos!=meshes_.end(); ++pos ) {
      if( pos->mesh_ == mesh ) {
         meshes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Triangle mesh is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single spring from the POV-Ray visualization.
 *
 * \param spring The spring to be removed.
 * \return void
 */
void Writer::removeSpring( ConstSpringID spring )
{
   for( Springs::Iterator pos=springs_.begin(); pos!=springs_.end(); ++pos ) {
      if( pos->spring_ == spring ) {
         springs_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Spring is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************




//=================================================================================================
//
//  VISUALIZATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief POV-Ray visualization of the visible rigid bodies.
 *
 * \return void
 * \exception std::runtime_error Error opening POV-Ray file.
 *
 * This function is automatically called every time step. It creates a new POV-Ray visualization
 * file for the current state of all registered rigid bodies using an automatically created file
 * name according to the specified file name template and the current internal file counter. If
 * no template was specified, the default file name template "image%.pov" is used.
 *
 * \see Writer::setFilename
 */
void Writer::trigger()
{
   // Skipping the visualization in case the time step has not yet reached the initial
   // visualization step or in case the time step is past the last visualization step
   if( TimeStep::step() < start_ || TimeStep::step() > end_ ) return;

   // Skipping the visualization for intermediate time steps
   if( ++steps_ < spacing_ ) return;

   // Setting the file name
   std::ostringstream file;
   file << prefix_ << counter_ << postfix_;

   // Adjusting the counters
   steps_ = 0;
   ++counter_;

   // Writing the POV-Ray visualization file
   writeFile( file.str().c_str() );
}
//*************************************************************************************************




//=================================================================================================
//
//  TEXTURE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the texture of a sphere.
 *
 * \param sphere The sphere to be textured.
 * \param texture The texture for the sphere.
 * \return void
 *
 * This function sets the POV-Ray appearance for the given sphere \a sphere.
 */
void Writer::setTexture( ConstSphereID sphere, const Texture& texture )
{
   for( Spheres::Iterator pos=spheres_.begin(); pos!=spheres_.end(); ++pos ) {
      if( pos->sphere_ == sphere ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a box.
 *
 * \param box The box to be textured.
 * \param texture The texture for the box.
 * \return void
 *
 * This function sets the POV-Ray appearance for the given box \a box.
 */
void Writer::setTexture( ConstBoxID box, const Texture& texture )
{
   for( Boxes::Iterator pos=boxes_.begin(); pos!=boxes_.end(); ++pos ) {
      if( pos->box_ == box ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Box is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a capsule.
 *
 * \param capsule The capsule to be textured.
 * \param texture The texture for the capsule.
 * \return void
 *
 * This function sets the POV-Ray appearance for the capsule \a capsule.
 */
void Writer::setTexture( ConstCapsuleID capsule, const Texture& texture )
{
   for( Capsules::Iterator pos=capsules_.begin(); pos!=capsules_.end(); ++pos ) {
      if( pos->capsule_ == capsule ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Capsule is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a cylinder.
 *
 * \param cylinder The cylinder to be textured.
 * \param texture The texture for the cylinder.
 * \return void
 *
 * This function sets the POV-Ray appearance for the cylinder \a cylinder.
 */
void Writer::setTexture( ConstCylinderID cylinder, const Texture& texture )
{
   for( Cylinders::Iterator pos=cylinders_.begin(); pos!=cylinders_.end(); ++pos ) {
      if( pos->cylinder_ == cylinder ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Cylinder is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a plane.
 *
 * \param plane The plane to be textured.
 * \param texture The texture for the plane.
 * \return void
 *
 * This function sets the POV-Ray appearance for the plane \a plane.
 */
void Writer::setTexture( ConstPlaneID plane, const Texture& texture )
{
   for( Planes::Iterator pos=planes_.begin(); pos!=planes_.end(); ++pos ) {
      if( pos->plane_ == plane ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Plane is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Setting the texture of a triangle mesh.
 *
 * \param mesh The triangle mesh to be textured.
 * \param texture The texture for the sphere.
 * \return void
 *
 * This function sets the POV-Ray appearance for the given sphere \a sphere.
 */
void Writer::setTexture( ConstTriangleMeshID mesh, const Texture& texture )
{
   for( Meshes::Iterator pos=meshes_.begin(); pos!=meshes_.end(); ++pos ) {
      if( pos->mesh_ == mesh ) {
         pos->default_ = false;
         pos->texture_ = texture;
         return;
      }
   }

   pe_INTERNAL_ASSERT( false, "Triangle mesh is not registered for the POV-Ray visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of all rigid bodies contained in a union.
 *
 * \param u The union to be textured.
 * \param texture The texture for all rigid bodies contained in the union.
 * \return void
 *
 * This function sets the POV-Ray appearance for all rigid bodies contained in union \a u.
 */
void Writer::setTexture( ConstUnionID u, const Texture& texture )
{
   // Setting the texture of all contained spheres
   {
      Union::ConstCastIterator<Sphere> begin( u->begin<Sphere>() );
      Union::ConstCastIterator<Sphere> end  ( u->end<Sphere>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }

   // Setting the texture of all contained boxes
   {
      Union::ConstCastIterator<Box> begin( u->begin<Box>() );
      Union::ConstCastIterator<Box> end  ( u->end<Box>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }

   // Setting the texture of all contained capsules
   {
      Union::ConstCastIterator<Capsule> begin( u->begin<Capsule>() );
      Union::ConstCastIterator<Capsule> end  ( u->end<Capsule>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }

   // Setting the texture of all contained cylinders
   {
      Union::ConstCastIterator<Cylinder> begin( u->begin<Cylinder>() );
      Union::ConstCastIterator<Cylinder> end  ( u->end<Cylinder>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }

   // Setting the texture of all contained planes
   {
      Union::ConstCastIterator<Plane> begin( u->begin<Plane>() );
      Union::ConstCastIterator<Plane> end  ( u->end<Plane>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }

   // Setting the texture of all contained triangle meshes
   //TODO
   /*{
      Union::ConstCastIterator<TriangleMesh> begin( u->begin<TriangleMesh>() );
      Union::ConstCastIterator<TriangleMesh> end  ( u->end<TriangleMesh>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }*/

   // Setting the texture of all contained unions
   {
      Union::ConstCastIterator<Union> begin( u->begin<Union>() );
      Union::ConstCastIterator<Union> end  ( u->end<Union>() );
      for( ; begin!=end; ++begin )
         setTexture( *begin, texture );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief POV-Ray visualizing of the registered rigid bodies.
 *
 * \param filename The file name for the POV-Ray file.
 * \return void
 * \exception std::runtime_error Invalid function call inside exclusive section.
 * \exception std::runtime_error Error opening POV-Ray file.
 *
 * This function can be used to create a single POV-Ray image. It creates a new POV-Ray
 * visualization file for the current state of all registered rigid bodies using the given
 * file name.
 *
 * \b Note: This function must not be called from inside an exclusive section. Calling this
 * function inside an exclusive section results in a \a std::runtime_error exception!
 */
void Writer::writeFile( const boost::filesystem::path& filename )
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
      throw std::runtime_error( "Directory for POV-Ray files does not exist" );
   if( file.empty() )
      throw std::runtime_error( "Invalid file name" );

   // Writing the POV-Ray header including the include files, the declarations, the camera,
   // the light sources, the background and the infinite rigid bodies. This task is solely
   // performed by the root process.
   if( MPISettings::rank() == MPISettings::root() )
   {
      // Opening the output file
      std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::trunc );
      if( !out.is_open() ) {
         std::ostringstream oss;
         oss << " Error opening POV-Ray file '" << filename << "' !\n";
         throw std::runtime_error( oss.str() );
      }

      // Writing the information header
      out << std::setiosflags(std::ios::left)
          << "/*==========================================================\n"
          << "== POV-Ray file for scene visualization of time step " << std::setw(4) << TimeStep::step() << " ==\n"
          << "== " << std::setw(54) << getTime() << " ==\n"
          << "==========================================================*/\n\n\n"
          << std::resetiosflags(std::ios::left);

      // Adding the include files
      if( !includes_.isEmpty() )
      {
         out << "/*============\n"
             << "== Includes ==\n"
             << "============*/\n\n";

         for( Includes::ConstIterator inc=includes_.begin(); inc!=includes_.end(); ++inc )
            out << "#include \"" << *inc << "\"\n";
         out << "\n\n";
      }

      // Adding declared identifiers
      if( !declarations_.isEmpty() )
      {
         out << "/*================\n"
             << "== Declarations ==\n"
             << "================*/\n\n";

         for( Declarations::ConstIterator dec=declarations_.begin(); dec!=declarations_.end(); ++dec )
            out << "#declare " << dec->key_ << " =\n" << dec->declared_ << "\n\n";
         out << "\n";
      }

      if( decorations_ ) {
         // Setting the camera position and orientation
         out << "/*===================================\n"
             << "== Camera position and orientation ==\n"
             << "===================================*/\n"
             << "// Camera aspect ratio: 4/3\n"
             << "//  => Up-angle=26,565, Up/Down-angle=53,13, Right-angle=33,69, Left/Right-angle=67,38\n\n";

         theCamera()->print( out, true );
         out << "\n\n";

         // Setting the light sources
         if( !lightsources_.isEmpty() )
         {
            out << "/*=================\n"
               << "== Light sources ==\n"
               << "=================*/\n\n";
            for( LightSources::ConstIterator ls=lightsources_.begin(); ls!=lightsources_.end(); ++ls )
               out << *ls << "\n\n";
            out << "\n";
         }

         // Setting the background color
         out << "/*==============\n"
             << "== Background ==\n"
             << "==============*/\n\n";

         out << "background { " << background_ << " }\n\n\n";
      }

      // Writing the rigid body header
      out << "/*================\n"
          << "== Rigid bodies ==\n"
          << "================*/\n\n";

      // Including the process-specific POV-Ray files
      if( !singleFile && MPISettings::size() > 1 )
      {
         for( int i=0; i<MPISettings::size(); ++i ) {
            path dir( lexical_cast<std::string>( i ) );
            dir /= file;
            out << "#include \"./" << dir.string() << "\"\n";
         }

         out << "\n";
      }

      // Writing the registered (infinite) planes
      for( Planes::ConstIterator p=planes_.begin(); p!=planes_.end(); ++p )
         writePlane( out, p->plane_, p->texture_ );

      // Closing the output file
      out.close();
   }

   // Generation of a single POV-Ray file
   // In case the 'singleFile' flag is set to 'true' all processes append their local, finite
   // rigid bodies to the main POV-Ray file 'filename'. This task is performed serially in
   // ascending order.
   if( singleFile || MPISettings::size() == 1 )
   {
      pe_SERIALIZATION
      {
         // Opening the output file
         std::ofstream out( filename.string().c_str(), std::ofstream::out | std::ostream::app );
         if( !out.is_open() ) {
            std::ostringstream oss;
            oss << " Error opening POV-Ray file '" << filename << "' !\n";
            throw std::runtime_error( oss.str() );
         }

         // Writing the registered spheres
         for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
            writeSphere( out, s->sphere_, s->texture_ );

         // Writing the registered boxes
         for( Boxes::ConstIterator b=boxes_.begin(); b!=boxes_.end(); ++b )
            writeBox( out, b->box_, b->texture_ );

         // Writing the registered capsules
         for( Capsules::ConstIterator c=capsules_.begin(); c!=capsules_.end(); ++c )
            writeCapsule( out, c->capsule_, c->texture_ );

         // Writing the registered cylinders
         for( Cylinders::ConstIterator c=cylinders_.begin(); c!=cylinders_.end(); ++c )
            writeCylinder( out, c->cylinder_, c->texture_ );

         // Writing the registered triangle meshes
         for( Meshes::ConstIterator m=meshes_.begin(); m!=meshes_.end(); ++m )
            writeMesh( out, m->mesh_, m->texture_ );

         // Writing the registered springs
         for( Springs::ConstIterator s=springs_.begin(); s!=springs_.end(); ++s )
            writeSpring( out, s->spring_ );

         // Closing the output file
         out.close();
      }
   }

   // Generation of multiple POV-Ray files
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
         oss << " Error opening POV-Ray file '" << filename << "' !\n";
         throw std::runtime_error( oss.str() );
      }

      // Writing the information header
      out << std::setiosflags(std::ios::left)
          << "/*==================================================================\n"
          << "== POV-Ray include file for scene visualization of time step " << std::setw(4) << TimeStep::step() << " ==\n"
          << "== " << std::setw(62) << getTime() << " ==\n"
          << "==================================================================*/\n\n\n"
          << std::resetiosflags(std::ios::left);

      // Writing the registered spheres
      for( Spheres::ConstIterator s=spheres_.begin(); s!=spheres_.end(); ++s )
         writeSphere( out, s->sphere_, s->texture_ );

      // Writing the registered boxes
      for( Boxes::ConstIterator b=boxes_.begin(); b!=boxes_.end(); ++b )
         writeBox( out, b->box_, b->texture_ );

      // Writing the registered capsules
      for( Capsules::ConstIterator c=capsules_.begin(); c!=capsules_.end(); ++c )
         writeCapsule( out, c->capsule_, c->texture_ );

      // Writing the registered cylinders
      for( Cylinders::ConstIterator c=cylinders_.begin(); c!=cylinders_.end(); ++c )
         writeCylinder( out, c->cylinder_, c->texture_ );

      // Writing the registered triangle meshes
      for( Meshes::ConstIterator m=meshes_.begin(); m!=meshes_.end(); ++m )
         writeMesh( out, m->mesh_, m->texture_ );

      // Writing the registered springs
      for( Springs::ConstIterator s=springs_.begin(); s!=springs_.end(); ++s )
         writeSpring( out, s->spring_ );

      // Closing the output file
      out.close();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given sphere.
 *
 * \param os Reference to the output stream.
 * \param sphere The sphere to be visualized.
 * \param texture The texture of the sphere.
 * \return void
 */
void Writer::writeSphere( std::ostream& os, ConstSphereID sphere, const Texture& texture )
{
   // Don't write an invisible or remote sphere
   if( !sphere->isVisible() || sphere->isRemote() ) return;

   // Only the root process should write global spheres
   if( sphere->isGlobal() && MPISettings::rank() != MPISettings::root() ) return;

   // Writing the sphere header
   os << "// Sphere " << sphere->getID() << " (sid=" << sphere->getSystemID() << ")\n";

   // Writing the sphere parameters
   const Vec3& pos( sphere->getPosition() );
   const Vec3 euler( calcEulerAngles( sphere->getRotation() ) );

   os << "sphere {\n"
      << "   <0,0,0>, " << sphere->getRadius() << "\n";

   texture.print( os, "   ", true );

   os << "   rotate " << euler << "\n"
      << "   translate <" << pos[0] << "," << pos[2] << "," << pos[1] << ">\n";

   os << "}\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given box.
 *
 * \param os Reference to the output stream.
 * \param box The box to be visualized.
 * \param texture The texture of the box.
 * \return void
 */
void Writer::writeBox( std::ostream& os, ConstBoxID box, const Texture& texture )
{
   // Don't write an invisible or remote box
   if( !box->isVisible() || box->isRemote() ) return;

   // Only the root process should write global boxes
   if( box->isGlobal() && MPISettings::rank() != MPISettings::root() ) return;

   // Writing the box header
   os << "// Box " << box->getID() << " (sid=" << box->getSystemID() << ")\n";

   // Writing the box parameters
   const Vec3& pos( box->getPosition() );
   const Vec3& lengths( box->getLengths() );
   const real xrange( static_cast<real>(0.5)*lengths[0] );
   const real yrange( static_cast<real>(0.5)*lengths[1] );
   const real zrange( static_cast<real>(0.5)*lengths[2] );
   const Vec3 euler( calcEulerAngles( box->getRotation() ) );

   os << "box {\n"
      << "   <" << -xrange << "," << -yrange << "," << -zrange << ">, <" << xrange << "," << yrange << "," << zrange << ">\n";

   texture.print( os, "   ", true );

   os << "   rotate " << euler << "\n"
      << "   translate <" << pos[0] << "," << pos[2] << "," << pos[1] << ">\n";

   os << "}\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given capsule.
 *
 * \param os Reference to the output stream.
 * \param capsule The capsule to be visualized.
 * \param texture The texture of the capsule.
 * \return void
 */
void Writer::writeCapsule( std::ostream& os, ConstCapsuleID capsule, const Texture& texture )
{
   // Don't write an invisible or remote capsule
   if( !capsule->isVisible() || capsule->isRemote() ) return;

   // Only the root process should write global capsules
   if( capsule->isGlobal() && MPISettings::rank() != MPISettings::root() ) return;

   // Writing the capsule header
   os << "// Capsule " << capsule->getID() << " (sid=" << capsule->getSystemID() << ")\n";

   // Writing the capsule parameters
   const Vec3& pos( capsule->getPosition() );
   const real radius( capsule->getRadius() );
   const real dx( static_cast<real>(0.5)*capsule->getLength() );
   const Vec3 euler( calcEulerAngles( capsule->getRotation() ) );

   os << "merge {\n";

   os << "   sphere {\n"
      << "      <" << -dx << ",0,0>, " << radius << "\n";
   texture.print( os, "      ", true );
   os << "   }\n";

   os << "   cylinder {\n"
      << "      <" << -dx << ",0,0>, <" << dx << ",0,0>, " << radius << "\n";
   texture.print( os, "      ", true );
   os << "   }\n";

   os << "   sphere {\n"
      << "      <" << dx << ",0,0>, " << radius << "\n";
   texture.print( os, "      ", true );
   os << "   }\n";

   os << "   rotate " << euler << "\n"
      << "   translate <" << pos[0] << "," << pos[2] << "," << pos[1] << ">\n";

   os << "}\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given cylinder.
 *
 * \param os Reference to the output stream.
 * \param cylinder The cylinder to be visualized.
 * \param texture The texture of the cylinder.
 * \return void
 */
void Writer::writeCylinder( std::ostream& os, ConstCylinderID cylinder, const Texture& texture )
{
   // Don't write an invisible or remote cylinder
   if( !cylinder->isVisible() || cylinder->isRemote() ) return;

   // Writing the cylinder header
   os << "// Cylinder " << cylinder->getID() << " (sid=" << cylinder->getSystemID() << ")\n";

   // Only the root process should write global cylinders
   if( cylinder->isGlobal() && MPISettings::rank() != MPISettings::root() ) return;

   // Writing the cylinder parameters
   const Vec3& pos( cylinder->getPosition() );
   const real radius( cylinder->getRadius() );
   const real dx( static_cast<real>(0.5)*cylinder->getLength() );
   const Vec3 euler( calcEulerAngles( cylinder->getRotation() ) );

   os << "cylinder {\n"
      << "   <" << -dx << ",0,0>, <" << dx << ",0,0>, " << radius << "\n";

   texture.print( os, "   ", true );

   os << "   rotate " << euler << "\n"
      << "   translate <" << pos[0] << "," << pos[2] << "," << pos[1] << ">\n";

   os << "}\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given plane.
 *
 * \param os Reference to the output stream.
 * \param plane The plane to be visualized.
 * \param texture The texture of the plane.
 * \return void
 */
void Writer::writePlane( std::ostream& os, ConstPlaneID plane, const Texture& texture )
{
   // Don't write an invisible plane
   if( !plane->isVisible() ) return;

   // Writing the plane header
   os << "// Plane " << plane->getID() << " (sid=" << plane->getSystemID() << ")\n";

   // Writing the plane parameters
   const Vec3& normal( plane->getNormal() );

   os << "plane {\n"
      << "   <" << normal[0] << "," << normal[2] << "," << normal[1] << ">, " << plane->getDisplacement() << "\n";

   texture.print( os, "   ", true );

   os << "}\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given triangle mesh.
 *
 * \param os Reference to the output stream.
 * \param mesh The triangle mesh to be visualized.
 * \param texture The texture of the triangle mesh.
 * \return void
 */
void Writer::writeMesh( std::ostream& os, ConstTriangleMeshID mesh, const Texture& texture )
{
   // Don't write an invisible or remote triangle mesh
   if( !mesh->isVisible() || mesh->isRemote() ) return;

   // Only the root process should write global triangle meshes
   if( mesh->isGlobal() && MPISettings::rank() != MPISettings::root() ) return;

   os << std::setiosflags(std::ios::left)
      << "/*======================\n"
      << "== Triangle mesh " << std::setw(4) << mesh->getID() << " ==" << "\n"
      << "======================*/\n\n"
      << std::resetiosflags(std::ios::left);

   // Writing the trinangle mesh header
   os << "// Triangle mesh " << mesh->getID() << " (sid=" << mesh->getSystemID() << ")\n";

   //Variabbles to match the pe and povRay coordinate systems
   const Vec3 euler( calcEulerAngles( mesh->getRotation() ) );
   const Vec3& pos( mesh->getPosition() );
   const Vec3 povPos(pos[0], pos[2], pos[1]);

   mesh->printPOVmesh2(os, texture, euler, povPos);

   os << "\n\n";

}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the POV-Ray output for the given spring.
 *
 * \param os Reference to the output stream.
 * \param spring The spring to be visualized.
 * \return void
 */
void Writer::writeSpring( std::ostream& os, ConstSpringID spring )
{
   // Don't write an invisible spring
   if( !spring->isVisible() ) return;

   // Writing the spring header
   os << "// Spring between body " << spring->getBody1()->getID()
      << " and " << spring->getBody2()->getID() << "\n";

   // Writing the spring parameters
   const Vec3 pos1( spring->getAnchor1WF() );
   const Vec3 pos2( spring->getAnchor2WF() );

   os << "cylinder {\n"
      << "   <" << pos1[0] << "," << pos1[2] << "," << pos1[1] << ">, <" << pos2[0] << "," << pos2[2] << "," << pos2[1] << ">, 0.1\n"
      << "}\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray writer.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Writer::print( std::ostream& os ) const
{
   std::ostringstream oss;
   oss << prefix_ << "%" << postfix_;

   os << " File names       = " << oss.str() << "\n"
      << " Next file        = " << counter_ << "\n"
      << " Background color = " << background_ << "\n";

   if( !includes_.isEmpty() ) {
      os << " Included files:\n";
      for( Includes::ConstIterator it=includes_.begin(); it!=includes_.end(); ++it )
         os << "   \"" << *it << "\"\n";
   }

   if( lightsources_.isEmpty() ) {
      os << " No light sources specified\n";
   }
   else {
      os << " Light sources:\n";
      for( LightSources::ConstIterator ls=lightsources_.begin(); ls!=lightsources_.end(); ++ls )
         os << "   " << *ls << "\n";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Activation of the POV-Ray writer.
 * \ingroup povray_writer
 *
 * \return Handle to the active POV-Ray writer.
 * \exception std::runtime_error Invalid activation of POV-Ray writer inside exclusive section.
 *
 * This function activates the POV-Ray writer for a POV-Ray visualization. The first call to
 * this function will activate the POV-Ray writer according to the given arguments and return
 * a handle to the active writer. Subsequent calls will ignore the arguments and only return
 * a handle to the POV-Ray writer. Note that this function must not be used inside an exclusive
 * section. The attempt to call this function from inside an exclusive section will result in a
 * \a std::runtime_error exception (for more details on exclusive sections please see the
 * pe::pe_EXCLUSIVE_SECTION description).
 */
WriterID activateWriter()
{
   if( ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid activation of POV-Ray writer inside exclusive section" );

   boost::mutex::scoped_lock lock( Writer::instanceMutex_ );
   static WriterID pov( new Writer() );
   return pov;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for POV-Ray writers.
 * \ingroup povray_writer
 *
 * \param os Reference to the output stream.
 * \param pov Reference to a constant POV-Ray writer object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Writer& pov )
{
   os << "--" << pe_BROWN << "POVRAY WRITER PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------\n";
   pov.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for POV-Ray writer handles.
 * \ingroup povray_writer
 *
 * \param os Reference to the output stream.
 * \param pov POV-Ray writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const WriterID& pov )
{
   os << "--" << pe_BROWN << "POVRAY WRITER PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------\n";
   pov->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant POV-Ray writer handles.
 * \ingroup povray_writer
 *
 * \param os Reference to the output stream.
 * \param pov Constant POV-Ray writer handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstWriterID& pov )
{
   os << "--" << pe_BROWN << "POVRAY WRITER PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------\n";
   pov->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
