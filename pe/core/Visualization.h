//=================================================================================================
/*!
 *  \file pe/core/Visualization.h
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

#ifndef _PE_CORE_VISUALIZATION_H_
#define _PE_CORE_VISUALIZATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Trigger.h>
#include <pe/core/Types.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Interface class for visualization purposes.
 * \ingroup core
 *
 * The Visualization class is an interface for visualization classes. Rigid bodies (spheres,
 * boxes, capsules and planes) created during the course of the simulation are automatically
 * registered for visualization. Derived visualization classes can access the registered rigid
 * bodies and are notified whenever any new bodies are created. Due to this mechanism, rigid
 * bodies don't need to be registered manually for visualization by the user and no destroyed
 * or invalid rigid bodies are visualized.
 */
class Visualization : public Trigger
{
protected:
   //**Type definitions****************************************************************************
   typedef PtrVector<Visualization,NoDelete>       Viewer;     //!< Container for active viewers.
   typedef PtrVector<const Sphere,NoDelete>        Spheres;    //!< Container for visible spheres.
   typedef PtrVector<const Box,NoDelete>           Boxes;      //!< Container for visible boxes.
   typedef PtrVector<const Capsule,NoDelete>       Capsules;   //!< Container for visible capsules.
   typedef PtrVector<const Cylinder,NoDelete>      Cylinders;  //!< Container for visible cylinders.
   typedef PtrVector<const Plane,NoDelete>         Planes;     //!< Container for visible planes.
   typedef PtrVector<const TriangleMesh,NoDelete>  Meshes;     //!< Container for visible meshes.
   typedef PtrVector<const Spring,NoDelete>        Springs;    //!< Container for visible springs.
   //**********************************************************************************************

public:
   //**Registration functions**********************************************************************
   /*!\name Registration functions */
   //@{
   static void add( ConstSphereID       sphere   );
   static void add( ConstBoxID          box      );
   static void add( ConstCapsuleID      capsule  );
   static void add( ConstCylinderID     cylinder );
   static void add( ConstPlaneID        plane    );
   static void add( ConstTriangleMeshID mesh     );
   static void add( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Deregistration functions********************************************************************
   /*!\name Deregistration functions */
   //@{
   static void remove( ConstSphereID       sphere   );
   static void remove( ConstBoxID          box      );
   static void remove( ConstCapsuleID      capsule  );
   static void remove( ConstCylinderID     cylinder );
   static void remove( ConstPlaneID        plane    );
   static void remove( ConstTriangleMeshID mesh     );
   static void remove( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Notification functions**********************************************************************
   /*!\name Notification functions */
   //@{
   static void changeVisibility( ConstSphereID       sphere   );
   static void changeVisibility( ConstBoxID          box      );
   static void changeVisibility( ConstCapsuleID      capsule  );
   static void changeVisibility( ConstCylinderID     cylinder );
   static void changeVisibility( ConstPlaneID        plane    );
   static void changeVisibility( ConstTriangleMeshID mesh     );
   static void changeVisibility( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Visualization();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Visualization();
   //@}
   //**********************************************************************************************

   //**Add functions*******************************************************************************
   /*!\name Add functions */
   //@{
   virtual void addSphere  ( ConstSphereID       sphere   ) = 0;
   virtual void addBox     ( ConstBoxID          box      ) = 0;
   virtual void addCapsule ( ConstCapsuleID      capsule  ) = 0;
   virtual void addCylinder( ConstCylinderID     cylinder ) = 0;
   virtual void addPlane   ( ConstPlaneID        plane    ) = 0;
   virtual void addMesh    ( ConstTriangleMeshID mesh     ) = 0;
   virtual void addSpring  ( ConstSpringID       spring   ) = 0;
   //@}
   //**********************************************************************************************

   //**Remove functions****************************************************************************
   /*!\name Remove functions */
   //@{
   virtual void removeSphere  ( ConstSphereID       sphere   ) = 0;
   virtual void removeBox     ( ConstBoxID          box      ) = 0;
   virtual void removeCapsule ( ConstCapsuleID      capsule  ) = 0;
   virtual void removeCylinder( ConstCylinderID     cylinder ) = 0;
   virtual void removePlane   ( ConstPlaneID        plane    ) = 0;
   virtual void removeMesh    ( ConstTriangleMeshID mesh     ) = 0;
   virtual void removeSpring  ( ConstSpringID       spring   ) = 0;
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
   virtual void changeSpringVisibility  ( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Trigger functions***************************************************************************
   /*!\name Trigger functions */
   //@{
   virtual void trigger() = 0;
   //@}
   //**********************************************************************************************

   //**Access functions****************************************************************************
   /*!\name Access functions */
   //@{
   static inline Spheres::Iterator   beginSpheres();
   static inline Spheres::Iterator   endSpheres();
   static inline Boxes::Iterator     beginBoxes();
   static inline Boxes::Iterator     endBoxes();
   static inline Capsules::Iterator  beginCapsules();
   static inline Capsules::Iterator  endCapsules();
   static inline Cylinders::Iterator beginCylinders();
   static inline Cylinders::Iterator endCylinders();
   static inline Planes::Iterator    beginPlanes();
   static inline Planes::Iterator    endPlanes();
   static inline Meshes::Iterator    beginMeshes();
   static inline Meshes::Iterator    endMeshes();
   static inline Springs::Iterator   beginSprings();
   static inline Springs::Iterator   endSprings();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static Viewer viewer_;        //!< The currently active viewers/visualization objects.
   static Spheres spheres_;      //!< The registered visible spheres.
   static Boxes boxes_;          //!< The registered visible boxes.
   static Capsules capsules_;    //!< The registered visible capsules.
   static Cylinders cylinders_;  //!< The registered visible cylinders.
   static Planes planes_;        //!< The registered visible planes.
   static Meshes meshes_;        //!< The registered visible triangle meshes.
   static Springs springs_;      //!< The registered visible springs.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a sphere iterator to the first visible sphere.
 *
 * \return Sphere iterator to the first visible sphere.
 */
inline Visualization::Spheres::Iterator Visualization::beginSpheres()
{
   return spheres_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a sphere iterator just past the last visible sphere.
 *
 * \return Sphere iterator just past the last visible sphere.
 */
inline Visualization::Spheres::Iterator Visualization::endSpheres()
{
   return spheres_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a box iterator to the first visible box.
 *
 * \return Box iterator to the first visible box.
 */
inline Visualization::Boxes::Iterator Visualization::beginBoxes()
{
   return boxes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a box iterator just past the last visible box.
 *
 * \return Box iterator just past the last visible box.
 */
inline Visualization::Boxes::Iterator Visualization::endBoxes()
{
   return boxes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a capsule iterator to the first visible capsule.
 *
 * \return Capsule iterator to the first visible capsule.
 */
inline Visualization::Capsules::Iterator Visualization::beginCapsules()
{
   return capsules_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a capsule iterator just past the last visible capsule.
 *
 * \return Capsule iterator just past the last visible capsule.
 */
inline Visualization::Capsules::Iterator Visualization::endCapsules()
{
   return capsules_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a cylinder iterator to the first visible cylinder.
 *
 * \return Cylinder iterator to the first visible cylinder.
 */
inline Visualization::Cylinders::Iterator Visualization::beginCylinders()
{
   return cylinders_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a cylinder iterator just past the last visible cylinder.
 *
 * \return Cylinder iterator just past the last visible cylinder.
 */
inline Visualization::Cylinders::Iterator Visualization::endCylinders()
{
   return cylinders_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a plane iterator to the first visible plane.
 *
 * \return Plane iterator to the first visible plane.
 */
inline Visualization::Planes::Iterator Visualization::beginPlanes()
{
   return planes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a plane iterator just past the last visible plane.
 *
 * \return Plane iterator just past the last visible plane.
 */
inline Visualization::Planes::Iterator Visualization::endPlanes()
{
   return planes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a mesh iterator to the first visible triangle mesh.
 *
 * \return Mesh iterator to the first visible triangle mesh.
 */
inline Visualization::Meshes::Iterator Visualization::beginMeshes()
{
   return meshes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a mesh iterator just past the last visible triangle mesh.
 *
 * \return Mesh iterator just past the last visible triangle mesh.
 */
inline Visualization::Meshes::Iterator Visualization::endMeshes()
{
   return meshes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a spring iterator to the first visible spring.
 *
 * \return Spring iterator to the first visible spring.
 */
inline Visualization::Springs::Iterator Visualization::beginSprings()
{
   return springs_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a spring iterator just past the last visible spring.
 *
 * \return Spring iterator just past the last visible spring.
 */
inline Visualization::Springs::Iterator Visualization::endSprings()
{
   return springs_.end();
}
//*************************************************************************************************

} // namespace pe

#endif
