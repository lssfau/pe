//=================================================================================================
/*!
 *  \file src/irrlicht/Viewer.cpp
 *  \brief Source file for the Irrlicht visualization viewer
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


#if HAVE_IRRLICHT

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <stdexcept>
#include <string>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/MPISettings.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/irrlicht/Viewer.h>
#include <pe/util/Logging.h>


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <irrlicht/irrlicht.h>


//*************************************************************************************************
// Macro definitions
//*************************************************************************************************

#define PE_STRINGIZE(x) #x


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool Viewer::active_( false );
boost::mutex Viewer::instanceMutex_;




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The constructor of the Viewer class.
 *
 * \param type Type of the renderer (software or hardware).
 * \param xsize x-size of the visualization window \f$ [1..\infty) \f$.
 * \param ysize y-size of the visualization window \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid window size.
 * \exception std::runtime_error Initialization error.
 *
 * The constructor handles the initial setup of the Irrlicht engine. As default, a FPS camera
 * at position (0,0,0) looking into direction (0,1,0) is added to the scene. However, initially
 * no lights are active.
 */
Viewer::Viewer( DriverType type, unsigned int xsize, unsigned int ysize )
   : Visualization()                // Initialization of the Visualization base object
   , Dependency<logging::Logger>()  // Initialization of the logger lifetime dependency
   , spacing_ ( 1 )                 // Spacing between two visualized time steps
   , steps_   ( 0 )                 // Time step counter between two time steps
   , device_  ( 0 )                 // Irrlicht visualization device
   , driver_  ( 0 )                 // Irrlicht video driver
   , smgr_    ( 0 )                 // Irrlicht scene manager
   , parent_  ( 0 )                 // Irrlicht scene transformation node
   , sphere_  ( 0 )                 // Triangle mesh for spheres
   , cylinder_( 0 )                 // Triangle mesh for capsules and cylinders
   , plane_   ( 0 )                 // Triangle mesh for planes
   , nodes_   ()                    // Active Irrlicht scene nodes
{
   using ::irr::f32;
   using ::irr::u32;
   using ::irr::core::dimension2d;
   using ::irr::video::E_DRIVER_TYPE;

   // Checking the sizes of the visualization window
   if( xsize == 0 || ysize == 0 )
      throw std::invalid_argument( "Invalid window size" );

   // Creating a Irrlicht device
   device_ = ::irr::createDevice( static_cast<E_DRIVER_TYPE>( type ), dimension2d<u32>( xsize, ysize ) );

   if( device_ == 0 )
      throw std::runtime_error( "Irrlicht device could not be created" );

   // Setting the active flag
   pe_INTERNAL_ASSERT( !active_, "Multiple constructor calls for a singleton class" );
   active_ = true;

   // Setting the video driver and the scene manager
   driver_ = device_->getVideoDriver();
   smgr_   = device_->getSceneManager();

   // Adding a FPS camera to the scene
   addFPSCamera( 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F );

   // Loading of the rigid body meshes
   const std::string mediaPath( PE_STRINGIZE( PE_MEDIA_PATH ) );
   const std::string cylinderMesh( mediaPath + "cylinder.3ds" );

   sphere_ = smgr_->addSphereMesh( "sphere", 1.0F );
   pe_INTERNAL_ASSERT( sphere_ != 0, "Irrlicht sphere mesh could not be created" );

   cylinder_ = smgr_->getMesh( cylinderMesh.c_str() );
   pe_INTERNAL_ASSERT( cylinder_ != 0, "Irrlicht cylinder mesh could not be created" );

   plane_ = smgr_->addHillPlaneMesh( "plane", dimension2d<f32>( 10.0F, 10.0F ),
                                              dimension2d<u32>( 50, 50 ),
                                              0, 0,
                                              dimension2d<f32>( 0, 0 ),
                                              dimension2d<f32>( 50, 50 ) );
   pe_INTERNAL_ASSERT( plane_ != 0, "Irrlicht plane mesh could not be created" );

   // Creating a parent node
   parent_ = smgr_->addDummyTransformationSceneNode();

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

   // Adding the registered visible springs
   for( Visualization::Springs::Iterator s=beginSprings(); s!=endSprings(); ++s )
      addSpring( *s );

   // Logging the successful setup of the Irrlicht viewer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the Irrlicht viewer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the Viewer class.
 */
Viewer::~Viewer()
{
   device_->drop();

   // Logging the successful destruction of the Irrlicht viewer
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the Irrlicht viewer instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the spacing of the real time visualization.
 *
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid spacing value.
 */
void Viewer::setSpacing( unsigned int spacing )
{
   // Checking the spacing value
   if( spacing_ == 0 )
      throw std::invalid_argument( "Invalid spacing value" );

   spacing_ = spacing;
}
//*************************************************************************************************




//=================================================================================================
//
//  CAMERA FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a static Irrlicht camera.
 *
 * \param px x-position of the static camera.
 * \param py y-position of the static camera.
 * \param pz z-position of the static camera.
 * \param lx x-position of the focus point of the camera (the target).
 * \param ly y-position of the focus point of the camera (the target).
 * \param lz z-position of the focus point of the camera (the target).
 * \return void
 * \exception std::runtime_error Camera initialization error.
 *
 * This function creates a static, non-moving camera at position (\a px, \a py, \a pz) that
 * looks into the direction of (\a lx, \a ly, \a lz). This camera replaces the currently
 * active camera.
 */
Camera Viewer::addStaticCamera( float px, float py, float pz, float lx, float ly, float lz )
{
   using ::irr::core::vector3df;
   using ::irr::scene::ICameraSceneNode;

   // Creating the static camera
   ICameraSceneNode* camera = smgr_->addCameraSceneNode( 0, vector3df(px,pz,py), vector3df(lx,lz,ly) );

   if( camera == 0 )
      throw std::runtime_error( "Irrlicht camera could not be created!" );

   // Hiding the cursor
   device_->getCursorControl()->setVisible( false );

   return Camera( camera );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a First-Person-Shooter (FPS) camera.
 *
 * \param px x-position of the FPS camera.
 * \param py y-position of the FPS camera.
 * \param pz z-position of the FPS camera.
 * \param lx x-position of the focus point of the camera (the target).
 * \param ly y-position of the focus point of the camera (the target).
 * \param lz z-position of the focus point of the camera (the target).
 * \param moveSpeed Speed in units per millisecond with which the camera is moved.
 * \return void
 * \exception std::runtime_error Camera initialization error.
 *
 * This function creates a First-Person-Shooter (FPS) camera at position (\a px, \a py, \a pz)
 * that looks into the direction of (\a lx, \a ly, \a lz). This camera replaces the currently
 * active camera. The following image shows the handling of the FPS camera:
 *
 * \image html fpscamera.png
 * \image latex fpscamera.eps "FPS camera" width=480pt
 */
Camera Viewer::addFPSCamera( float px, float py, float pz, float lx, float ly, float lz,
                             float moveSpeed )
{
   using ::irr::core::vector3df;
   using ::irr::scene::ICameraSceneNode;

   // Generating a key map for the FPS camera
   ::irr::SKeyMap keyMap[10];
   keyMap[0].Action  = ::irr::EKA_MOVE_FORWARD;
   keyMap[0].KeyCode = ::irr::KEY_UP;
   keyMap[1].Action  = ::irr::EKA_MOVE_FORWARD;
   keyMap[1].KeyCode = ::irr::KEY_KEY_W;

   keyMap[2].Action  = ::irr::EKA_MOVE_BACKWARD;
   keyMap[2].KeyCode = ::irr::KEY_DOWN;
   keyMap[3].Action  = ::irr::EKA_MOVE_BACKWARD;
   keyMap[3].KeyCode = ::irr::KEY_KEY_S;

   keyMap[4].Action  = ::irr::EKA_STRAFE_LEFT;
   keyMap[4].KeyCode = ::irr::KEY_LEFT;
   keyMap[5].Action  = ::irr::EKA_STRAFE_LEFT;
   keyMap[5].KeyCode = ::irr::KEY_KEY_A;

   keyMap[6].Action  = ::irr::EKA_STRAFE_RIGHT;
   keyMap[6].KeyCode = ::irr::KEY_RIGHT;
   keyMap[7].Action  = ::irr::EKA_STRAFE_RIGHT;
   keyMap[7].KeyCode = ::irr::KEY_KEY_D;

   keyMap[8].Action  = ::irr::EKA_JUMP_UP;
   keyMap[8].KeyCode = ::irr::KEY_SPACE;
   keyMap[9].Action  = ::irr::EKA_CROUCH;
   keyMap[9].KeyCode = ::irr::KEY_CONTROL;

   // Creating the FPS camera
   ICameraSceneNode* camera = smgr_->addCameraSceneNodeFPS( 0, 50.0F, moveSpeed, -1, keyMap, 10 );

   if( camera == 0 )
      throw std::runtime_error( "Irrlicht camera could not be created!" );

   // Setting the position and orientation of the camera
   camera->setPosition( vector3df( px, pz, py ) );
   camera->setTarget( vector3df( lx, lz, ly ) );

   // Hiding the cursor
   device_->getCursorControl()->setVisible( false );

   return Camera( camera );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a Maya camera.
 *
 * \param px x-position of the Maya camera.
 * \param py y-position of the Maya camera.
 * \param pz z-position of the Maya camera.
 * \param lx x-position of the focus point of the camera (the target).
 * \param ly y-position of the focus point of the camera (the target).
 * \param lz z-position of the focus point of the camera (the target).
 * \return void
 * \exception std::runtime_error Camera initialization error.
 *
 * This function creates a Maya camera at position (\a px, \a py, \a pz) that looks into the
 * direction of (\a lx, \a ly, \a lz). This camera replaces the currently active camera. The
 * following image shows the handling of the Maya camera:
 *
 * \image html mayacamera.png
 * \image latex mayacamera.eps "FPS camera" width=620pt
 */
Camera Viewer::addMayaCamera( float px, float py, float pz, float lx, float ly, float lz )
{
   using ::irr::core::vector3df;
   using ::irr::scene::ICameraSceneNode;

   // Creating the Maya camera
   ICameraSceneNode* camera = smgr_->addCameraSceneNodeMaya( 0, -90.0F, 100.0F, 50.0F );

   if( camera == 0 )
      throw std::runtime_error( "Irrlicht camera could not be created!" );

   // Setting the position and orientation of the camera
   camera->setPosition( vector3df( px, pz, py ) );
   camera->setTarget( vector3df( lx, lz, ly ) );

   // Setting the cursor visible
   device_->getCursorControl()->setVisible( true );

   return Camera( camera );
}
//*************************************************************************************************




//=================================================================================================
//
//  LIGHT SOURCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of an Irrlicht point light source.
 *
 * \param px x-position of the light source.
 * \param py y-position of the light source.
 * \param pz z-position of the light source.
 * \param red The red color channel of the light source \f$ [0..1] \f$.
 * \param green The green color channel of the light source \f$ [0..1] \f$.
 * \param blue The blue color channel of the light source \f$ [0..1] \f$.
 * \param radius The influence radius of the light source \f$ (0..\infty) \f$.
 * \return The new light source.
 *
 * This function creates an Irrlicht light source at position (\a px, \a py, \a pz). This
 * light source emits light of the specified color (\a red, \a green, \a blue) uniformly
 * in all directions. All color channels have to be specified as single-precision floating
 * point values between 0 and 1. The following code shows some examples:

   \code
   ViewerID viewer;
   viewer->addLightSource( 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F );  // A bright white light source at (0,0,0)
   viewer->addLightSource( 1.0F, 2.0F, 3.0F, 0.0F, 0.5F, 0.0F );  // A medium green light source at (1,2,3)
   \endcode
 */
LightSource Viewer::addPointLightSource( float px, float py, float pz,
                                    float red, float green, float blue, float radius )
{
   using ::irr::core::vector3df;
   using ::irr::scene::ILightSceneNode;
   using ::irr::video::SColorf;

   ILightSceneNode* light = smgr_->addLightSceneNode( 0, vector3df( px, pz, py ),
                                                      SColorf( red, green, blue ), radius );
   return LightSource( light );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of an Irrlicht directional light source.
 *
 * \param px x-position of the light source.
 * \param py y-position of the light source.
 * \param pz z-position of the light source.
 * \param dx x-component of light direction.
 * \param dy y-component of light direction.
 * \param dz z-component of light direction.
 * \param red The red color channel of the light source \f$ [0..1] \f$.
 * \param green The green color channel of the light source \f$ [0..1] \f$.
 * \param blue The blue color channel of the light source \f$ [0..1] \f$.
 * \param radius The influence radius of the light source \f$ (0..\infty) \f$.
 * \return The new light source.
 *
 * This function creates an Irrlicht light source at position (\a px, \a py, \a pz). This
 * light source emits light of the specified color (\a red, \a green, \a blue) uniformly
 * in all directions. All color channels have to be specified as single-precision floating
 * point values between 0 and 1. The following code shows some examples:

   \code
   ViewerID viewer;
   viewer->addLightSource( 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F );  // A bright white light source at (0,0,0)
   viewer->addLightSource( 1.0F, 2.0F, 3.0F, 0.0F, 0.5F, 0.0F );  // A medium green light source at (1,2,3)
   \endcode
 */
LightSource Viewer::addDirectionalLightSource( float px, float py, float pz, float dx, float dy, float dz,
                                    float red, float green, float blue, float radius )
{
   using ::irr::core::vector3df;
   using ::irr::scene::ILightSceneNode;
   using ::irr::video::SColorf;
   using ::irr::core::radToDeg;

   ILightSceneNode* light = smgr_->addLightSceneNode( 0, vector3df( px, py, pz ),
                                                      SColorf( red, green, blue ), radius );

   light->setLightType(::irr::video::ELT_DIRECTIONAL);
   // The initial light direction vector is (0, 0, 1) pointing in z direction.

   // The pe y-axis coincides with the z-axis of Irrlicht and the other way round.
   std::swap(dy, dz);

   float dlen( sqrt( dx * dx + dy * dy + dz * dz ) );

   // Compute the angle between the xz-plane and the direction vector d, via the angle between
   // xz-plane normal and d: -( 90° - acos((n * d) / (|n| * |d|))).
   // This angle is the first Euler angle (rotation around the x axis). Positive angles mean
   // rotation in counter-clockwise direction.
   float angleX( acosf( dy / dlen ) - float( M_PI_2 ) );

   // Compute the angle between the initial light direction (0,0,1) and the direction vector d
   // projected to the xz-plane: -( atan2(dz, dx) - pi/2 )
   float angleY = float( M_PI_2 ) - atan2f(dz, dx);
   //std::cout << radToDeg(angleX) << " " << radToDeg(angleY) << std::endl;

   // Rotate initial light direction around x axis and then around original y-axis (xyz Euler angles).
   light->setRotation(vector3df( radToDeg(angleX), radToDeg(angleY), 0.0F ));

   // Postcondition: vector3df( radToDeg(angleX), radToDeg(angleY), 0.0F ).rotationToDirection(vector3df(0,0,1)) should match the specified light direction.

   return LightSource( light );
}
//*************************************************************************************************




//=================================================================================================
//
//  TEXTURE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a monochrome color texture.
 *
 * \param red The red color channel of the texture.
 * \param green The green color channel of the texture.
 * \param blue The blue color channel of the texture.
 * \return The new texture.
 *
 * This function creates a new monochrome texture with the color (\a red, \a green, \a blue).
 * This texture can be used in the setTexture() functions to specify the appearance of a specific
 * rigid body in the visualization.
 */
Texture Viewer::createTexture( unsigned char red, unsigned char green, unsigned char blue ) const
{
   using ::irr::u32;
   using ::irr::core::dimension2d;
   using ::irr::video::ECF_R8G8B8;
   using ::irr::video::IImage;
   using ::irr::video::ITexture;

   unsigned char color[3] = { red, green, blue };
   IImage* tmp   = driver_->createImageFromData( ECF_R8G8B8, dimension2d<u32>(1,1), &color );
   ITexture* tex = driver_->addTexture( "texture", tmp );
   tmp->drop();

   return tex;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Loading of a texture from file.
 *
 * \param file File name of the texture.
 * \return The loaded texture.
 * \exception std::invalid_argument Texture could not be loaded.
 *
 * This function reads the texture from file \a file and returns it. This texture can be
 * used in the setTexture() functions to specify the appearance of a specific rigid body
 * in the visualization.
 */
Texture Viewer::createTexture( const char* const file ) const
{
   using ::irr::video::ITexture;

   ITexture* tex = driver_->getTexture( file );
   if( tex == 0 ) throw std::invalid_argument( "Texture could not be loaded!" );
   return tex;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a sphere.
 *
 * \param sphere The sphere primitive.
 * \param texture The texture for the sphere.
 * \return void
 *
 * This function sets the texture of a sphere in the real time visualization. The specified
 * texture replaces the current texture of the sphere and deactivates the wireframe mode.
 */
void Viewer::setTexture( ConstSphereID sphere, Texture texture )
{
   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n ) {
      if( n->body_ == sphere )
      {
         n->node_->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
         n->node_->setMaterialTexture( 0, texture );
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a box.
 *
 * \param box The box primitive.
 * \param texture The texture for the box.
 * \return void
 *
 * This function sets the texture of a box in the real time visualization. The specified
 * texture replaces the current texture of the box and deactivates the wireframe mode.
 */
void Viewer::setTexture( ConstBoxID box, Texture texture )
{
   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n ) {
      if( n->body_ == box )
      {
         n->node_->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
         n->node_->setMaterialTexture( 0, texture );
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a capsule.
 *
 * \param capsule The capsule primitive.
 * \param texture The texture of the capsule.
 * \return void
 *
 * This function sets the texture of a capsule in the real time visualization. The specified
 * texture replaces the current texture of the capsule and deactivates the wireframe mode.
 */
void Viewer::setTexture( ConstCapsuleID capsule, Texture texture )
{
   typedef ::irr::scene::ISceneNode       SceneNode;
   typedef ::irr::core::list<SceneNode*>  List;

   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n )
   {
      if( n->body_ == capsule )
      {
         const List& children( n->node_->getChildren() );
         for( List::ConstIterator c=children.begin(); c!=children.end(); ++c ) {
            (*c)->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
            (*c)->setMaterialTexture( 0, texture );
         }
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a cylinder.
 *
 * \param cylinder The cylinder primitive.
 * \param texture The texture of the cylinder.
 * \return void
 *
 * This function sets the texture of a cylinder in the real time visualization. The specified
 * texture replaces the current texture of the cylinder and deactivates the wireframe mode.
 */
void Viewer::setTexture( ConstCylinderID cylinder, Texture texture )
{
   typedef ::irr::scene::ISceneNode       SceneNode;
   typedef ::irr::core::list<SceneNode*>  List;

   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n )
   {
      if( n->body_ == cylinder )
      {
         const List& children( n->node_->getChildren() );
         for( List::ConstIterator c=children.begin(); c!=children.end(); ++c ) {
            (*c)->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
            (*c)->setMaterialTexture( 0, texture );
         }
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a plane.
 *
 * \param plane The plane primitive.
 * \param texture The texture of the plane.
 * \return void
 *
 * This function sets the texture of a plane in the real time visualization. The specified
 * texture replaces the current texture of the plane and deactivates the wireframe mode.
 */
void Viewer::setTexture( ConstPlaneID plane, Texture texture )
{
   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n ) {
      if( n->body_ == plane )
      {
         n->node_->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
         n->node_->setMaterialTexture( 0, texture );
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of a triangle mesh.
 *
 * \param mesh The triangle mesh primitive.
 * \param texture The texture for the triangle mesh.
 * \return void
 *
 * This function sets the texture of a triangle mesh in the real time visualization. The
 * specified texture replaces the current texture of the triangle mesh and deactivates
 * the wireframe mode.
 */
void Viewer::setTexture( ConstTriangleMeshID mesh, Texture texture )
{
   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n ) {
      if( n->body_ == mesh )
      {
         n->node_->setMaterialFlag( ::irr::video::EMF_WIREFRAME, false );
         n->node_->setMaterialTexture( 0, texture );
         return;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the texture of all rigid bodies contained in a union.
 *
 * \param u The union compound geometry.
 * \param texture The texture for all rigid bodies contained in the union.
 * \return void
 *
 * This function sets the texture of all rigid bodies contained in union \a u. The specified
 * texture replaces the current texture of the all bodies in the union and deactivates the
 * wireframe mode.
 */
void Viewer::setTexture( ConstUnionID u, Texture texture )
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

   // TODO: triangle meshes
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Activating the wireframe mode of a rigid body.
 *
 * \param body The rigid body.
 * \return void
 *
 * This function activates the wireframe mode of a given rigid body. The body will be displayed
 * as a wireframe model in the real time visualization.
 */
void Viewer::setWireframe( ConstBodyID body )
{
   for( Nodes::iterator n=nodes_.begin(); n!=nodes_.end(); ++n ) {
      if( n->body_ == body )
      {
         n->node_->setMaterialFlag( ::irr::video::EMF_WIREFRAME, true );
         break;
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single sphere for the Irrlicht visualization.
 *
 * \param sphere The sphere to be registered.
 * \return void
 */
void Viewer::addSphere( ConstSphereID sphere )
{
   using ::irr::f32;
   using ::irr::core::vector3df;

   // Converting the radius, the global position and the orientation
   // of the sphere to Irrlicht data types
   const real  radius( sphere->getRadius()   );
   const Vec3& gpos  ( sphere->getPosition() );
   const Vec3  euler ( calcEulerAngles( sphere->getRotation() ) );

   const vector3df r( static_cast<f32>( radius   ),
                      static_cast<f32>( radius   ),
                      static_cast<f32>( radius   ) );
   const vector3df p( static_cast<f32>( gpos[0]  ),
                      static_cast<f32>( gpos[2]  ),
                      static_cast<f32>( gpos[1]  ) );
   const vector3df e( static_cast<f32>( euler[0] ),
                      static_cast<f32>( euler[1] ),
                      static_cast<f32>( euler[2] ) );

   // Creating a new animated mesh scene node
   Node node;
   node.body_ = sphere;
   node.node_ = smgr_->addAnimatedMeshSceneNode( sphere_, parent_, -1, p, e, r );


   // Setting the visibility of the scene node
   if( !sphere->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single box for the Irrlicht visualization.
 *
 * \param box The box to be registered.
 * \return void
 */
void Viewer::addBox( ConstBoxID box )
{
   using ::irr::f32;
   using ::irr::core::vector3df;

   // Converting the side lengths, the global position and the orientation
   // of the box to Irrlicht data types
   const Vec3& length( box->getLengths() );
   const Vec3& gpos  ( box->getPosition() );
   const Vec3  euler ( calcEulerAngles( box->getRotation() ) );

   const vector3df l( static_cast<f32>( length[0] ),
                      static_cast<f32>( length[2] ),
                      static_cast<f32>( length[1] ) );
   const vector3df p( static_cast<f32>( gpos[0]   ),
                      static_cast<f32>( gpos[2]   ),
                      static_cast<f32>( gpos[1]   ) );
   const vector3df e( static_cast<f32>( euler[0]  ),
                      static_cast<f32>( euler[1]  ),
                      static_cast<f32>( euler[2]  ) );

   // Creating a new animated mesh scene node
   Node node;
   node.body_ = box;
   node.node_ = smgr_->addCubeSceneNode( 1.0F, parent_, -1, p, e, l );

   // Setting the visibility of the scene node
   if( !box->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single capsule for the Irrlicht visualization.
 *
 * \param capsule The capsule to be registered.
 * \return void
 */
void Viewer::addCapsule( ConstCapsuleID capsule )
{
   using ::irr::f32;
   using ::irr::core::vector3df;
   using ::irr::scene::ISceneNode;

   // Converting the radius, the length, the global position and the
   // orientation of the capsule to Irrlicht data types
   const Vec3& gpos ( capsule->getPosition() );
   const Vec3  euler( calcEulerAngles( capsule->getRotation() ) );

   const f32       r( static_cast<f32>( capsule->getRadius() ) );
   const f32       l( static_cast<f32>( capsule->getLength()*real(0.5) ) );
   const vector3df p( static_cast<f32>( gpos[0]  ),
                      static_cast<f32>( gpos[2]  ),
                      static_cast<f32>( gpos[1]  ) );
   const vector3df e( static_cast<f32>( euler[0] ),
                      static_cast<f32>( euler[1] ),
                      static_cast<f32>( euler[2] ) );

   // Creating a new animated mesh scene node
   Node node;
   node.body_ = capsule;
   node.node_ = smgr_->addEmptySceneNode( parent_, -1 );

   smgr_->addAnimatedMeshSceneNode( cylinder_, node.node_, -1, vector3df(  0, 0, 0 ),
                                                               vector3df(  0, 0, 0 ),
                                                               vector3df(  l, r, r ) );
   smgr_->addAnimatedMeshSceneNode( sphere_,   node.node_, -1, vector3df(  l, 0, 0 ),
                                                               vector3df(  0, 0, 0 ),
                                                               vector3df(  r, r, r ) );
   smgr_->addAnimatedMeshSceneNode( sphere_,   node.node_, -1, vector3df( -l, 0, 0 ),
                                                               vector3df(  0, 0, 0 ),
                                                               vector3df(  r, r, r ) );

   node.node_->setPosition( p );
   node.node_->setRotation( e );

   // Setting the visibility of the scene node
   if( !capsule->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single cylinder for the Irrlicht visualization.
 *
 * \param cylinder The cylinder to be registered.
 * \return void
 */
void Viewer::addCylinder( ConstCylinderID cylinder )
{
   using ::irr::f32;
   using ::irr::core::vector3df;

   // Converting the radius, the length, the global position and the
   // orientation of the cylinder to Irrlicht data types
   const Vec3& gpos  ( cylinder->getPosition() );
   const Vec3  euler ( calcEulerAngles( cylinder->getRotation() ) );

   const f32       r( static_cast<f32>( cylinder->getRadius() ) );
   const f32       l( static_cast<f32>( cylinder->getLength()*real(0.5) ) );
   const vector3df p( static_cast<f32>( gpos[0]  ),
                      static_cast<f32>( gpos[2]  ),
                      static_cast<f32>( gpos[1]  ) );
   const vector3df e( static_cast<f32>( euler[0] ),
                      static_cast<f32>( euler[1] ),
                      static_cast<f32>( euler[2] ) );

   // Creating a new animated mesh scene node
   Node node;
   node.body_ = cylinder;
   node.node_ = smgr_->addAnimatedMeshSceneNode( cylinder_, parent_, -1, p, e, vector3df( l, r, r ) );

   // Setting the visibility of the scene node
   if( !cylinder->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single plane for the Irrlicht visualization.
 *
 * \param plane The plane to be registered.
 * \return void
 */
void Viewer::addPlane( ConstPlaneID plane )
{
   using ::irr::f32;
   using ::irr::core::vector3df;

   // Converting the global position and the orientation of the plane in Irrlicht data types
   const Vec3& gpos ( plane->getPosition() );
   const Vec3  euler( calcEulerAngles( plane->getRotation() ) );

   const vector3df p( static_cast<f32>( gpos[0]  ),
                      static_cast<f32>( gpos[2]  ),
                      static_cast<f32>( gpos[1]  ) );
   const vector3df e( static_cast<f32>( euler[0] ),
                      static_cast<f32>( euler[1] ),
                      static_cast<f32>( euler[2] ) );

   // Creating a new animated mesh scene node
   Node node;
   node.body_ = plane;
   node.node_ = smgr_->addAnimatedMeshSceneNode( plane_, 0, -1, p, e );

   // Setting the visibility of the scene node
   if( !plane->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single triangle mesh for the Irrlicht visualization.
 *
 * \param mesh The triangle mesh to be registered.
 * \return void
 */
void Viewer::addMesh( ConstTriangleMeshID mesh )
{
   using ::irr::u16;
   using ::irr::f32;
   using ::irr::core::vector3df;
   using ::irr::scene::SAnimatedMesh;
   using ::irr::scene::SMesh;
   using ::irr::scene::SMeshBuffer;
   using ::irr::video::S3DVertex;
   using ::irr::video::SColor;

   //u16 index( 0 );
   //S3DVertex v;
   SMesh*         ms = new SMesh();
   SMeshBuffer*   mb = mesh->getIrrlichtCacheBuffer();//new SMeshBuffer();
   SAnimatedMesh* am = new SAnimatedMesh();

   /*
   // Setting the default color of the triangle mesh
   v.Color = SColor( 255, 255, 255, 255 );

   // Building the Irrlicht mesh geometrie
   // Note: the Irrlicht engine expects the vertices of the triangles in clockwise
   // (left-handed) order!
   for( TriangleMesh::Iterator t=mesh->begin(); t!=mesh->end(); ++t )
   {
      const Vec3& normal( t->getNormal() );
      v.Normal.X = static_cast<f32>( normal[0] );
      v.Normal.Y = static_cast<f32>( normal[2] );
      v.Normal.Z = static_cast<f32>( normal[1] );

      const Vec3& a( t->getRelA() );
      v.Pos.X = static_cast<f32>( a[0] );
      v.Pos.Y = static_cast<f32>( a[2] );
      v.Pos.Z = static_cast<f32>( a[1] );
      mb->Vertices.push_back( v );
      mb->Indices.push_back( index++ );

      const Vec3& c( t->getRelC() );
      v.Pos.X = static_cast<f32>( c[0] );
      v.Pos.Y = static_cast<f32>( c[2] );
      v.Pos.Z = static_cast<f32>( c[1] );
      mb->Vertices.push_back( v );
      mb->Indices.push_back( index++ );

      const Vec3& b( t->getRelB() );
      v.Pos.X = static_cast<f32>( b[0] );
      v.Pos.Y = static_cast<f32>( b[2] );
      v.Pos.Z = static_cast<f32>( b[1] );
      mb->Vertices.push_back( v );
      mb->Indices.push_back( index++ );
   }
*/
   // Configuring the mesh
   ms->addMeshBuffer( mb );
   ms->recalculateBoundingBox();

   // Configuring the animated mesh
   am->Type = ::irr::scene::EAMT_UNKNOWN;
   am->addMesh( ms );
   am->recalculateBoundingBox();

   // Dropping the mesh and the mesh buffer
   ms->drop();
   //mb->drop();

   // Converting the global position and the orientation
   // of the triangle mesh to Irrlicht data types
   const Vec3& gpos ( mesh->getPosition() );
   const Vec3  euler( calcEulerAngles( mesh->getRotation() ) );

   const vector3df p( static_cast<f32>( gpos[0]  ),
                      static_cast<f32>( gpos[2]  ),
                      static_cast<f32>( gpos[1]  ) );
   const vector3df e( static_cast<f32>( euler[0] ),
                      static_cast<f32>( euler[1] ),
                      static_cast<f32>( euler[2] ) );

   // Creating an animated mesh scene node
   Node node;
   node.body_ = mesh;
   node.node_ = smgr_->addAnimatedMeshSceneNode( am, parent_, -1, p, e, vector3df( 1.0F, 1.0F, 1.0F ) );

   // Setting the visibility of the scene node
   if( !mesh->isVisible() )
      node.node_->setVisible( false );

   // Registering the scene node
   nodes_.push_back( node );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a single spring for the Irrlicht visualization.
 *
 * \param spring The spring to be registered.
 * \return void
 */
void Viewer::addSpring( ConstSpringID /*spring*/ )
{}
//*************************************************************************************************




//=================================================================================================
//
//  REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Removing a single sphere from the Irrlicht visualization.
 *
 * \param sphere The sphere to be removed.
 * \return void
 */
void Viewer::removeSphere( ConstSphereID sphere )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == sphere ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single box from the Irrlicht visualization.
 *
 * \param box The box to be removed.
 * \return void
 */
void Viewer::removeBox( ConstBoxID box )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == box ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Box is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single capsule from the Irrlicht visualization.
 *
 * \param capsule The capsule to be removed.
 * \return void
 */
void Viewer::removeCapsule( ConstCapsuleID capsule )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == capsule ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Capsule is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single cylinder from the Irrlicht visualization.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 */
void Viewer::removeCylinder( ConstCylinderID cylinder )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == cylinder ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Cylinder is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single plane from the Irrlicht visualization.
 *
 * \param plane The plane to be removed.
 * \return void
 */
void Viewer::removePlane( ConstPlaneID plane )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == plane ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Plane is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single triangle mesh from the Irrlicht visualization.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 */
void Viewer::removeMesh( ConstTriangleMeshID mesh )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == mesh ) {
         pos->node_->remove();
         nodes_.erase( pos );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Triangle mesh is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a single spring from the Irrlicht visualization.
 *
 * \param spring The spring to be removed.
 * \return void
 */
void Viewer::removeSpring( ConstSpringID /*spring*/ )
{}
//*************************************************************************************************




//=================================================================================================
//
//  HANDLE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handling the visibility change of a sphere primitive.
 *
 * \param sphere The changed sphere primitive.
 * \return void
 */
void Viewer::changeSphereVisibility( ConstSphereID sphere )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == sphere ) {
         pos->node_->setVisible( sphere->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Sphere is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling the visibility change of a box primitive.
 *
 * \param box The changed box primitive.
 * \return void
 */
void Viewer::changeBoxVisibility( ConstBoxID box )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == box ) {
         pos->node_->setVisible( box->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Box is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling the visibility change of a capsule primitive.
 *
 * \param capsule The changed capsule primitive.
 * \return void
 */
void Viewer::changeCapsuleVisibility( ConstCapsuleID capsule )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == capsule ) {
         pos->node_->setVisible( capsule->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Capsule is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling the visibility change of a cylinder primitive.
 *
 * \param cylinder The changed cylinder primitive.
 * \return void
 */
void Viewer::changeCylinderVisibility( ConstCylinderID cylinder )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == cylinder ) {
         pos->node_->setVisible( cylinder->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "C is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling the visibility change of a plane primitive.
 *
 * \param plane The changed plane primitive.
 * \return void
 */
void Viewer::changePlaneVisibility( ConstPlaneID plane )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == plane ) {
         pos->node_->setVisible( plane->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Plane is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling the visibility change of a triangle mesh primitive.
 *
 * \param mesh The changed triangle mesh primitive.
 * \return void
 */
void Viewer::changeMeshVisibility( ConstTriangleMeshID mesh )
{
   for( Nodes::iterator pos=nodes_.begin(); pos!=nodes_.end(); ++pos ) {
      if( pos->body_ == mesh ) {
         pos->node_->setVisible( mesh->isVisible() );
         return;
      }
   }
   pe_INTERNAL_ASSERT( false, "Triangle mesh is not registered for the Irrlicht visualization" );
}
//*************************************************************************************************




//=================================================================================================
//
//  VISUALIZATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Update of the real time visualization.
 *
 * \return void
 *
 * This function is automatically called every time step. It triggers an update of the real
 * time visualization. This function causes the entire scene to be redrawn.
 */
void Viewer::trigger()
{
   using ::irr::f32;
   using ::irr::core::matrix4;
   using ::irr::core::quaternion;
   using ::irr::core::vector3df;
   using ::irr::video::SColor;

   // Skipping the visualization for intermediate time steps
   if( ++steps_ < spacing_ ) return;

   // Resetting the time step counter
   steps_ = 0;

   // Redrawing the scene
   if( device_->run() && device_->isWindowActive() )
   {
      // Visualization of the visible scene nodes
      for( Nodes::const_iterator n=nodes_.begin(); n!=nodes_.end(); ++n )
      {
         // Don't update invisible scene nodes
         if( !n->body_->isVisible() ) continue;

         // Setting the position and orientation of the scene nodes
         const Vec3& gpos ( n->body_->getPosition() );
         const Vec3  euler( calcEulerAngles( n->body_->getRotation() ) );

         n->node_->setPosition( vector3df( static_cast<f32>( gpos[0] ),
                                           static_cast<f32>( gpos[2] ),
                                           static_cast<f32>( gpos[1] ) ) );
         n->node_->setRotation( vector3df( static_cast<f32>( euler[0] ),
                                           static_cast<f32>( euler[1] ),
                                           static_cast<f32>( euler[2] ) ) );
      }

      // Initializing the video driver
      driver_->setTransform( ::irr::video::ETS_WORLD, matrix4() );

      // Starting the visualization
      driver_->beginScene( true, true, SColor( 0, 0, 0, 0 ) );

      // Visualization of the visible springs
      for( Springs::Iterator spring=beginSprings(); spring!=endSprings(); ++spring )
      {
         // Don't draw an invisible spring
         if( !spring->isVisible() ) continue;

         // Drawing a 3D-line for all visible springs
         const Vec3 pos1( spring->getAnchor1WF() );
         const Vec3 pos2( spring->getAnchor2WF() );

         driver_->draw3DLine( vector3df( static_cast<f32>( pos1[0] ),
                                         static_cast<f32>( pos1[2] ),
                                         static_cast<f32>( pos1[1] ) ),
                              vector3df( static_cast<f32>( pos2[0] ),
                                         static_cast<f32>( pos2[2] ),
                                         static_cast<f32>( pos2[1] ) ),
                              SColor( 0, 0, 0, 0 ) );
      }

      // Finalizing the visualization
      smgr_->drawAll();
      device_->getGUIEnvironment()->drawAll();
      driver_->endScene();

      // Writing the window caption
      const ::irr::s32 fps( driver_->getFPS() );

      wchar_t tmp[1024];
      swprintf( tmp, 1024, L"pe Physics Engine (%ls)(fps:%d)", driver_->getName(), fps );
      device_->setWindowCaption( tmp );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adds a coordinate system arrows to the scene.
 *
 * \param scale Scales the arrows.
 * \return void
 *
 * This function adds three arrows to the scene. A red arrow pointing from the origin in the
 * positive x direction, a green arrow in the y direction and a blue arrow in the z direction.
 * Note that since Irrlicht uses a left-handed coordinate system and pe and right-handed coordinate
 * system, the y and z axes are switched, so that e.g. the green arrow denotes the pe's z axes.
 * By default the arrows have unit length.
 */
void Viewer::displayCoordinateSystemFrame( float scale )
{

   using ::irr::video::SColor;
   using ::irr::core::vector3df;
   using ::irr::scene::IAnimatedMeshSceneNode;

   // Arrow mesh is initially oriented along (0,0,0) -> (0, length, 0)
   if( arrow_ == 0 )
      arrow_ = smgr_->addArrowMesh( "arrow", SColor(1.0F, 1.0F, 1.0F, 1.0F), SColor(1.0F, 1.0F, 1.0F, 1.0F), 8, 8, 1.0F, 0.7F, 0.05F, 0.1F );

   IAnimatedMeshSceneNode* axisX = smgr_->addAnimatedMeshSceneNode( arrow_, parent_, -1, vector3df(0,0,0), vector3df(0,0,-90), vector3df(scale, scale, scale) );
   IAnimatedMeshSceneNode* axisY = smgr_->addAnimatedMeshSceneNode( arrow_, parent_, -1, vector3df(0,0,0), vector3df(0,0,0),   vector3df(scale, scale, scale) );
   IAnimatedMeshSceneNode* axisZ = smgr_->addAnimatedMeshSceneNode( arrow_, parent_, -1, vector3df(0,0,0), vector3df(90,0,0),  vector3df(scale, scale, scale) );

   axisX->setMaterialTexture( 0, createTexture(255, 0, 0) );
   axisY->setMaterialTexture( 0, createTexture(0, 255, 0) );
   axisZ->setMaterialTexture( 0, createTexture(0, 0, 255) );
}
//*************************************************************************************************




//=================================================================================================
//
//  VIEWER ACTIVATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Activation of the Irrlicht viewer.
 * \ingroup irrlicht
 *
 * \param type Type of the renderer (software or hardware).
 * \param xsize x-size of the visualization window \f$ [1..\infty) \f$.
 * \param ysize y-size of the visualization window \f$ [1..\infty) \f$.
 * \return Handle to the active Irrlicht viewer.
 * \exception std::runtime_error Invalid activation of Irrlicht viewer in MPI parallel simulation.
 * \exception std::invalid_argument Invalid window size.
 * \exception std::runtime_error Initialization error.
 *
 * This function configures and actives the Irrlicht viewer. Depending on the used render type
 * the viewer will use software or hardware rendering for the real time visualization. The real
 * time visualization will be displayed in a window of size \f$ xsize \times ysize \f$.\n
 * It is only possible to create a single Irrlicht viewer due to limitations of the Irrlicht
 * visualization engine. It is also not possible to deactivate the viewer and to activate it
 * again with different options. Therefore the viewer will, once activated, stay active until
 * the end of the program. The first call to this function will activate the Irrlicht viewer
 * according to the given arguments and return the handle to the configured viewer. Subsequent
 * calls will ignore all arguments and only return the handle to the viewer.
 *
 * \b Note: The Irrlicht visualization cannot be used during MPI parallel simulations. Calling
 * this function while the total number of MPI processes is larger than 1 will result in a
 * \a std::runtime_error exception.
 */
ViewerID activateViewer( DriverType type, unsigned int xsize, unsigned int ysize )
{
   if( MPISettings::size() > 1 )
      throw std::runtime_error( "Invalid activation of Irrlicht viewer in MPI parallel simulation" );

   boost::mutex::scoped_lock lock( Viewer::instanceMutex_ );
   static ViewerID viewer( new Viewer( type, xsize, ysize ) );
   return viewer;
}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
