//=================================================================================================
/*!
 *  \file src/core/Visualization.cpp
 *  \brief Interface for visualization classes
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

#include <algorithm>
#include <pe/core/Visualization.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

Visualization::Viewer    Visualization::viewer_;
Visualization::Spheres   Visualization::spheres_;
Visualization::Boxes     Visualization::boxes_;
Visualization::Capsules  Visualization::capsules_;
Visualization::Cylinders Visualization::cylinders_;
Visualization::Planes    Visualization::planes_;
Visualization::Meshes    Visualization::meshes_;
Visualization::Springs   Visualization::springs_;




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Visualization class.
 */
Visualization::Visualization()
   : Trigger()  // Initialization of the Trigger base class
{
   viewer_.pushBack( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Visualization class.
 */
Visualization::~Visualization()
{
   Viewer::Iterator pos( std::find( viewer_.begin(), viewer_.end(), this ) );
   pe_INTERNAL_ASSERT( pos != viewer_.end(), "Visualization object is not registered!" );
   viewer_.erase( pos );
}
//*************************************************************************************************




//=================================================================================================
//
//  REGISTRATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registration function for a sphere.
 *
 * \param sphere The sphere to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstSphereID sphere )
{
   spheres_.pushBack( sphere );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addSphere( sphere );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a box.
 *
 * \param box The box to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstBoxID box )
{
   boxes_.pushBack( box );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addBox( box );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a capsule.
 *
 * \param capsule The capsule to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstCapsuleID capsule )
{
   capsules_.pushBack( capsule );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addCapsule( capsule );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a cylinder.
 *
 * \param cylinder The cylinder to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstCylinderID cylinder )
{
   cylinders_.pushBack( cylinder );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addCylinder( cylinder );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a plane.
 *
 * \param plane The plane to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstPlaneID plane )
{
   planes_.pushBack( plane );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addPlane( plane );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a triangle mesh.
 *
 * \param mesh The triangle mesh to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstTriangleMeshID mesh )
{
   meshes_.pushBack( mesh );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addMesh( mesh );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registration function for a spring.
 *
 * \param spring The spring to be registered for visualization.
 * \return void
 */
void Visualization::add( ConstSpringID spring )
{
   springs_.pushBack( spring );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->addSpring( spring );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DEREGISTRATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Deregistration function for a sphere.
 *
 * \param sphere The sphere to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstSphereID sphere )
{
   Spheres::Iterator pos( std::find( spheres_.begin(), spheres_.end(), sphere ) );
   pe_INTERNAL_ASSERT( pos != spheres_.end(), "Sphere is not registered for visualization" );
   spheres_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeSphere( sphere );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a box.
 *
 * \param box The box to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstBoxID box )
{
   Boxes::Iterator pos( std::find( boxes_.begin(), boxes_.end(), box ) );
   pe_INTERNAL_ASSERT( pos != boxes_.end(), "Box is not registered for visualization" );
   boxes_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeBox( box );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a capsule.
 *
 * \param capsule The capsule to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstCapsuleID capsule )
{
   Capsules::Iterator pos( std::find( capsules_.begin(), capsules_.end(), capsule ) );
   pe_INTERNAL_ASSERT( pos != capsules_.end(), "Capsule is not registered for visualization" );
   capsules_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeCapsule( capsule );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a cylinder.
 *
 * \param cylinder The cylinder to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstCylinderID cylinder )
{
   Cylinders::Iterator pos( std::find( cylinders_.begin(), cylinders_.end(), cylinder ) );
   pe_INTERNAL_ASSERT( pos != cylinders_.end(), "Cylinder is not registered for visualization" );
   cylinders_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeCylinder( cylinder );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a plane.
 *
 * \param plane The plane to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstPlaneID plane )
{
   Planes::Iterator pos( std::find( planes_.begin(), planes_.end(), plane ) );
   pe_INTERNAL_ASSERT( pos != planes_.end(), "Plane is not registered for visualization" );
   planes_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removePlane( plane );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a triangle mesh.
 *
 * \param mesh The triangle mesh to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstTriangleMeshID mesh )
{
   Meshes::Iterator pos( std::find( meshes_.begin(), meshes_.end(), mesh ) );
   pe_INTERNAL_ASSERT( pos != meshes_.end(), "Triangle mesh is not registered for visualization" );
   meshes_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeMesh( mesh );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistration function for a spring.
 *
 * \param spring The spring to be deregistered from visualization.
 * \return void
 */
void Visualization::remove( ConstSpringID spring )
{
   Springs::Iterator pos( std::find( springs_.begin(), springs_.end(), spring ) );
   pe_INTERNAL_ASSERT( pos != springs_.end(), "Spring is not registered for visualization" );
   springs_.erase( pos );

   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->removeSpring( spring );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  NOTIFICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a sphere primitive.
 *
 * \param sphere The changed sphere primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstSphereID sphere )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeSphereVisibility( sphere );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a box primitive.
 *
 * \param box The changed box primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstBoxID box )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeBoxVisibility( box );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a capsule primitive.
 *
 * \param capsule The changed capsule primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstCapsuleID capsule )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeCapsuleVisibility( capsule );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a cylinder primitive.
 *
 * \param cylinder The changed cylinder primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstCylinderID cylinder )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeCylinderVisibility( cylinder );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a plane primitive.
 *
 * \param plane The changed plane primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstPlaneID plane )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changePlaneVisibility( plane );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a triangle mesh primitive.
 *
 * \param mesh The changed triangle mesh primitive.
 * \return void
 */
void Visualization::changeVisibility( ConstTriangleMeshID mesh )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeMeshVisibility( mesh );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signaling the change of the visibility of a spring.
 *
 * \param spring The changed spring.
 * \return void
 */
void Visualization::changeVisibility( ConstSpringID spring )
{
   for( Viewer::Iterator v=viewer_.begin(); v!=viewer_.end(); ++v ) {
      v->changeSpringVisibility( spring );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn void Visualization::addSphere( ConstSphereID sphere )
 * \brief Adding a single sphere primitive to the visualization system.
 *
 * \param sphere The sphere to be added.
 * \return void
 *
 * This function adds a sphere primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addBox( ConstBoxID box )
 * \brief Adding a single box primitive to the visualization system.
 *
 * \param box The box to be added.
 * \return void
 *
 * This function adds a box primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addCapsule( ConstCapsuleID capsule )
 * \brief Adding a single capsule primitive to the visualization system.
 *
 * \param capsule The capsule to be added.
 * \return void
 *
 * This function adds a capsule primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addCylinder( ConstCylinderID cylinder )
 * \brief Adding a single cylinder primitive to the visualization system.
 *
 * \param cylinder The cylinder to be added.
 * \return void
 *
 * This function adds a cylinder primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addPlane( ConstPlaneID plane )
 * \brief Adding a single plane primitive to the visualization system.
 *
 * \param plane The plane to be added.
 * \return void
 *
 * This function adds a plane primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addMesh( ConstMeshID mesh )
 * \brief Adding a single triangle mesh primitive to the visualization system.
 *
 * \param mesh The triangle mesh to be added.
 * \return void
 *
 * This function adds a triangle mesh primitive to the visualization system and assigns it the
 * default appearance of the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::addSpring( ConstSpringID spring )
 * \brief Adding a single spring to the visualization system.
 *
 * \param spring The spring to be added.
 * \return void
 *
 * This function adds a spring to the visualization system and assigns it the default
 * appearance of the visualization system.
 */
//*************************************************************************************************




//=================================================================================================
//
//  REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn void Visualization::removeSphere( ConstSphereID sphere )
 * \brief Removing a single sphere primitive from the visualization system.
 *
 * \param sphere The sphere to be removed.
 * \return void
 *
 * This function completely removes a sphere primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removeBox( ConstBoxID box )
 * \brief Removing a single box primitive from the visualization system.
 *
 * \param box The box to be removed.
 * \return void
 *
 * This function completely removes a box primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removeCapsule( ConstCapsuleID capsule )
 * \brief Removing a single capsule primitive from the visualization system.
 *
 * \param capsule The capsule to be removed.
 * \return void
 *
 * This function completely removes a capsule primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removeCylinder( ConstCylinderID cylinder )
 * \brief Removing a single cylinder primitive from the visualization system.
 *
 * \param cylinder The cylinder to be removed.
 * \return void
 *
 * This function completely removes a cylinder primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removePlane( ConstPlaneID plane )
 * \brief Removing a single plane primitive from the visualization system.
 *
 * \param plane The plane to be removed.
 * \return void
 *
 * This function completely removes a plane primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removeMesh( ConstMeshID mesh )
 * \brief Removing a single triangle mesh primitive from the visualization system.
 *
 * \param mesh The triangle mesh to be removed.
 * \return void
 *
 * This function completely removes a triangle mesh primitive from the visualization system.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void Visualization::removeSpring( ConstSpringID spring )
 * \brief Removing a single spring from the visualization system.
 *
 * \param spring The spring to be removed.
 * \return void
 *
 * This function completely removes a spring from the visualization system.
 */
//*************************************************************************************************




//=================================================================================================
//
//  HANDLE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of spheres.
 *
 * \param sphere The changed sphere primitive.
 * \return void
 */
void Visualization::changeSphereVisibility( ConstSphereID /*sphere*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of boxes.
 *
 * \param box The changed box primitive.
 * \return void
 */
void Visualization::changeBoxVisibility( ConstBoxID /*box*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of capsules.
 *
 * \param capsule The changed capsule primitive.
 * \return void
 */
void Visualization::changeCapsuleVisibility( ConstCapsuleID /*capsule*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of cylinders.
 *
 * \param cylinder The changed cylinder primitive.
 * \return void
 */
void Visualization::changeCylinderVisibility( ConstCylinderID /*cylinder*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of planes.
 *
 * \param plane The changed plane primitive.
 * \return void
 */
void Visualization::changePlaneVisibility( ConstPlaneID /*plane*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of triangle meshes.
 *
 * \param mesh The changed triangle mesh primitive.
 * \return void
 */
void Visualization::changeMeshVisibility( ConstTriangleMeshID /*mesh*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the handle function for visibility changes of springs.
 *
 * \param spring The changed spring.
 * \return void
 */
void Visualization::changeSpringVisibility( ConstSpringID /*spring*/ )
{}
//*************************************************************************************************

} // namespace pe
