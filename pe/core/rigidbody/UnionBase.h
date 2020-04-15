//=================================================================================================
/*!
 *  \file pe/core/rigidbody/UnionBase.h
 *  \brief Base class for the union compound geometry
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

#ifndef _PE_CORE_RIGIDBODY_UNIONBASE_H_
#define _PE_CORE_RIGIDBODY_UNIONBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/BodyManager.h>
#include <pe/core/Link.h>
#include <pe/core/rigidbody/SuperBody.h>
#include <pe/core/Types.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for the union compound geometry.
 * \ingroup union
 *
 * The UnionBase class represents the base class for the union compound geometry. It provides
 * the basic functionality of a union. For a full description of the union compound geometry,
 * see the Union class description.
 */
class UnionBase : public SuperBody
                , public BodyManager
{
public:
   //**Type definitions****************************************************************************
   //! Container for the rigid bodies contained in the union compound geometry.
   typedef PtrVector<RigidBody,NoDelete>  Bodies;

   //! Container for links defined within the union compound geometry.
   typedef PtrVector<Link,NoDelete>  Links;

   typedef Bodies::SizeType       SizeType;           //!< Size type of the union.
   typedef Bodies::Iterator       Iterator;           //!< Iterator over the contained rigid bodies.
   typedef Bodies::ConstIterator  ConstIterator;      //!< Constant iterator over the contained rigid bodies.
   typedef Links::Iterator        LinkIterator;       //!< Iterator over the contained links.
   typedef Links::ConstIterator   ConstLinkIterator;  //!< Constant iterator over the contained links.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit UnionBase( id_t sid, id_t uid, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~UnionBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
   virtual void resetForce();
   //@}
   //**********************************************************************************************

   //**MPI functions*******************************************************************************
   /*!\name MPI functions */
   //@{
   virtual void setRemote( bool remote );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void clear();
   //@}
   //**********************************************************************************************

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void calcBoundingBox();   // Calculation of the axis-aligned bounding box
   void calcCenterOfMass();  // Calculation of the center of mass
   void calcInertia();       // Calculation of the moment of inertia
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Bodies bodies_;  //!< Rigid bodies contained in the union.
   Links  links_;   //!< Links contained in the union.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
