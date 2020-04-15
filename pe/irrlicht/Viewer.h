//=================================================================================================
/*!
 *  \file pe/irrlicht/Viewer.h
 *  \brief Header file for the Irrlicht visualization viewer
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

#ifndef _PE_IRRLICHT_VIEWER_H_
#define _PE_IRRLICHT_VIEWER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <boost/thread/mutex.hpp>
#include <pe/core/Types.h>
#include <pe/core/Visualization.h>
#include <pe/irrlicht/Camera.h>
#include <pe/irrlicht/LightSource.h>
#include <pe/irrlicht/Texture.h>
#include <pe/irrlicht/Types.h>
#include <pe/irrlicht/ViewerID.h>
#include <pe/math/Constants.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Dependency.h>


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <irrlicht/EDriverTypes.h>


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  ENUMERATIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Driver types for software or hardware real time visualization.
 * \ingroup irrlicht
 */
enum DriverType {
   software = ::irr::video::EDT_SOFTWARE,  //!< Software renderer.
                                           /*!< The software renderer runs on all platforms and
                                                with every hardware. It only supports a limited
                                                number of primitive 3d functions that are quite
                                                fast but very inaccurate. */
   opengl   = ::irr::video::EDT_OPENGL     //!< OpenGL hardware rendering.
                                           /*!< The OpenGL hardware renderer is available on most
                                                platforms. It accelerates rendering of 3D and 2D
                                                primitives. */
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup irrlicht_viewer Viewer
 * \ingroup irrlicht
 */
/*!\brief Irrlicht real time visualization viewer.
 * \ingroup irrlicht_viewer
 *
 * The viewer class is a wrapper class for the Irrlicht visualization engine. It provides all
 * necessary functionality to visualize a configuration of rigid bodies of the physics engine
 * in real time. The following example gives an impression of the use of the Viewer class:

   \code
   // These lines initialize the simulation setting: a sphere and a box are created such that
   // they will hit each other. All created (visible) rigid bodies are automatically registered
   // for the real time visualization.
   pe::WorldID  world  = pe::theWorld();
   pe::SphereID sphere = createSphere( 1, -5.0, 0.0, 0.0, 3.0, iron );
   pe::BoxID    box    = createBox( 2, 5.0, 0.0, 0.0, 6.0, 5.0, 4.0, iron );
   pe::PlaneID  plane  = createPlane( 3, 0.0, 0.0, 1.0, -10.0, iron );

   sphere->setLinearVel( 0.1, 0.0, 0.0 );

   box->rotate( -0.785, 0.785, 0.0 );
   box->setLinearVel( -0.1, 0.0, 0.0 );
   box->setAngularVel( 0.0, 0.0, 0.1 );

   // The following line activates the Irrlicht viewer. The visualization will be hardware
   // accelerated by OpenGL and the resolution of the visualization window is 800 x 600.
   pe::irrlicht::ViewerID viewer = pe::irrlicht::activateViewer( pe::irrlicht::opengl, 800, 600 );

   // The default camera is a FPS camera at positon (0,0,0). The next line replaces this
   // camera by a static camera at position (3,-6,3) looking at (0,0,0).
   viewer->addStaticCamera( 2.0F, -20.0F, 2.0F, 0.0F, 0.0F, 0.0F );

   // In order to see the scene it is necessary to create at least a singe light source.
   // The next line creates a light source at the initial position of the camera.
   // Additionally, the ambient light of the light source is set to (0.1,0.1,0.1).
   pe::irrlicht::LightSource light = viewer->addPointLightSource( 0.0F, 0.0F, 20.0F, 1.0F, 1.0F, 1.0F );
   light.setAmbient( 0.1F, 0.1F, 0.1F );

   // The appearance of the rigid bodies can be adapted with textures. In the next lines
   // two monochrome textures will be created and one texture will be loaded from file.
   peirrlicht::::Texture red   = viewer->createTexture( 255, 0, 0 );
   peirrlicht::::Texture blue  = viewer->createTexture( 0, 0, 255 );
   peirrlicht::::Texture chess = viewer->createTexture( "./pe/media/chess.png" );

   viewer->setTexture( sphere, red   );
   viewer->setTexture( box,    blue  );
   viewer->setTexture( plane,  chess );

   // The simulation loop for the created setup. The real time visualization is updated in
   // each time step. In order to increase the spacing between two updates, the setSpacing()
   // function can be used.
   for( unsigned int i=0; i<1000; ++i )
   {
      world->simulationStep( 0.1 );
   }
   \endcode

 * It is only possible to create a single Irrlicht viewer due to limitations of the Irrlicht
 * visualization engine. It is also not possible to deactivate the viewer and to activate
 * it again with different options. Therefore the viewer will, once activated, stay active
 * until the end of the program.
 */
class Viewer : public Visualization
             , private Dependency<logging::Logger>
{
private:
   //**Declarations for nested structures**********************************************************
   struct Node;
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef std::vector<Node>  Nodes;  //!< Vector for active Irrlicht scene nodes.
   /*! \endcond */
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Viewer( DriverType type, unsigned int xsize, unsigned int ysize );
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Viewer();
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setSpacing( unsigned int spacing );
   //@}
   //**********************************************************************************************

   //**Camera functions****************************************************************************
   /*!\name Camera functions */
   //@{
   Camera addStaticCamera( float px, float py, float pz, float lx, float ly, float lz );
   Camera addFPSCamera   ( float px, float py, float pz, float lx, float ly, float lz,
                           float moveSpeed=0.5F );
   Camera addMayaCamera  ( float px, float py, float pz, float lx, float ly, float lz );
   //@}
   //**********************************************************************************************

   //**Light source functions**********************************************************************
   /*!\name Light source functions */
   //@{
   LightSource addPointLightSource( float px, float py, float pz,
                               float red, float green, float blue, float radius=100.0F );
   LightSource addDirectionalLightSource( float px, float py, float pz, float dx, float dy, float dz,
                               float red, float green, float blue, float radius=100.0F );
   //@}
   //**********************************************************************************************

   //**Texture functions***************************************************************************
   /*!\name Texture functions */
   //@{
   Texture createTexture( unsigned char red, unsigned char green, unsigned char blue ) const;
   Texture createTexture( const char* const file ) const;

   void    setTexture( ConstSphereID       sphere  , Texture texture );
   void    setTexture( ConstBoxID          box     , Texture texture );
   void    setTexture( ConstCapsuleID      capsule , Texture texture );
   void    setTexture( ConstCylinderID     cylinder, Texture texture );
   void    setTexture( ConstPlaneID        plane   , Texture texture );
   void    setTexture( ConstTriangleMeshID mesh    , Texture texture );
   void    setTexture( ConstUnionID        u       , Texture texture );

   void    setWireframe( ConstBodyID body );
   //@}
   //**********************************************************************************************

   //**Visualization functions*********************************************************************
   /*!\name Visualization functions */
   //@{
   void displayCoordinateSystemFrame( float scale = 1.0F );
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

   //**Handle functions****************************************************************************
   /*!\name Handle functions */
   //@{
   virtual void changeSphereVisibility  ( ConstSphereID       sphere   );
   virtual void changeBoxVisibility     ( ConstBoxID          box      );
   virtual void changeCapsuleVisibility ( ConstCapsuleID      capsule  );
   virtual void changeCylinderVisibility( ConstCylinderID     cylinder );
   virtual void changePlaneVisibility   ( ConstPlaneID        plane    );
   virtual void changeMeshVisibility    ( ConstTriangleMeshID mesh     );
   //@}
   //**********************************************************************************************

   //**Visualization functions*********************************************************************
   /*!\name Visualization functions */
   //@{
   virtual void trigger();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const Vec3 calcEulerAngles( const Rot3& R ) const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   unsigned int spacing_;                                 //!< Spacing between two visualized time steps.
   unsigned int steps_;                                   //!< Time step counter between two time steps.
   ::irr::IrrlichtDevice* device_;                        //!< Irrlicht visualization device.
   ::irr::video::IVideoDriver* driver_;                   //!< Irrlicht video driver.
   ::irr::scene::ISceneManager* smgr_;                    //!< Irrlicht scene manager.
   ::irr::scene::IDummyTransformationSceneNode* parent_;  //!< Irrlicht scene transformation node.
   ::irr::scene::IAnimatedMesh* sphere_;                  //!< Triangle mesh for spheres.
   ::irr::scene::IAnimatedMesh* cylinder_;                //!< Triangle mesh for capsules and cylinders.
   ::irr::scene::IAnimatedMesh* plane_;                   //!< Triangle mesh for planes.
   ::irr::scene::IAnimatedMesh* arrow_;                   //!< Triangle mesh for coordinate axes arrows.
   Nodes nodes_;                                          //!< Active Irrlicht scene nodes.

   static bool active_;                 //!< Active flag of the Irrlicht viewer.
   static boost::mutex instanceMutex_;  //!< Synchronization mutex for access to the Irrlicht viewer.
   //@}
   //**********************************************************************************************

   //**Private struct Node*************************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Data structure for active Irrlicht scene nodes. */
   struct Node
   {
    public:
      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      ConstBodyID body_;                //!< A visible rigid body.
      ::irr::scene::ISceneNode* node_;  //!< The corresponding Irrlicht scene node.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Viewer activation functions*****************************************************************
   /*! \cond PE_INTERNAL */
   friend bool     isActive();
   friend ViewerID activateViewer( DriverType type, unsigned int xsize, unsigned int ysize );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  VIEWER ACTIVATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Viewer setup functions */
//@{
inline bool     isActive();
       ViewerID activateViewer( DriverType type, unsigned int xsize, unsigned int ysize );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the Irrlicht visualization is active or not.
 * \ingroup irrlicht
 *
 * \return \a true if the Irrlicht visualization is active, \a false if not.
 */
inline bool isActive()
{
   return Viewer::active_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the Euler angles (in degree).
 *
 * \param R The rotation of a rigid body.
 * \return The Euler angles for a rotation order of x, y, z.
 *
 * This function calculates the required Euler angles for the Irrlicht visualization. It
 * performs the necessary transformation for the right-handed pe coordinate system to the
 * left-handed Irrlicht coordinate system and calculates the Euler angles in degrees.
 */
inline const Vec3 Viewer::calcEulerAngles( const Rot3& R ) const
{
   static const real factor( -real(180)/M_PI );

   // Calculation of the left-handed Euler angles ( = factor * R.getEulerAngles( XZYs ) )
   const real cy( std::sqrt( R[0]*R[0] + R[6]*R[6] ) );
   if( cy > real(16)*std::numeric_limits<real>::epsilon() ) {
      return factor * Vec3( -std::atan2(  R[5], R[4] ),
                            -std::atan2( -R[3], cy   ),
                            -std::atan2(  R[6], R[0] ) );
   }
   else {
      return factor * Vec3( -std::atan2( -R[7], R[8] ),
                            -std::atan2( -R[3], cy   ),
                             real(0) );
   }
}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
