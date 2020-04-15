//=================================================================================================
/*!
 *  \file pe/core/Section.h
 *  \brief Header file for the Section class
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

#ifndef _PE_CORE_SECTION_H_
#define _PE_CORE_SECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/rigidbody/SuperBody.h>
#include <pe/core/Types.h>
#include <pe/math/Quaternion.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A section of a link between two rigid bodies.
 * \ingroup core
 *
 * A section is part of a link within a union. For each link, the superordinate union is
 * logically split in two sections, for which the forces and torques acting in the link are
 * calculated in each time step.
 *
 * \image html section.png
 * \image latex section.eps "A section" width=200pt
 */
class Section : public SuperBody
{
private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class Link;
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   //! Rigid body container for the setup of a section.
   typedef PtrVector<RigidBody,NoDelete>  Bodies;
   //**********************************************************************************************

private:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Section( BodyID sb );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Section();
   //@}
   //**********************************************************************************************

public:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool containsRelPoint ( real px, real py, real pz ) const;
   virtual bool containsRelPoint ( const Vec3& rpos )          const;
   virtual bool containsPoint    ( real px, real py, real pz ) const;
   virtual bool containsPoint    ( const Vec3& gpos )          const;
   virtual bool isSurfaceRelPoint( real px, real py, real pz ) const;
   virtual bool isSurfaceRelPoint( const Vec3& rpos )          const;
   virtual bool isSurfacePoint   ( real px, real py, real pz ) const;
   virtual bool isSurfacePoint   ( const Vec3& gpos )          const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible    ( bool visible );
   virtual void setPosition   ( real px, real py, real pz );
   virtual void setPosition   ( const Vec3& gpos );
   virtual void setOrientation( real r, real i, real j, real k );
   virtual void setOrientation( const Quat& q );
   //@}
   //**********************************************************************************************

   //**Translation functions***********************************************************************
   /*!\name Translation functions */
   //@{
   virtual void translate( real dx, real dy, real dz );
   virtual void translate( const Vec3& dp );
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions */
   //@{
   virtual void rotate( real x, real y, real z, real angle );
   virtual void rotate( const Vec3& axis, real angle );
   virtual void rotate( real xangle, real yangle, real zangle );
   virtual void rotate( const Vec3& euler );
   virtual void rotate( const Quat& dq );

   virtual void rotateAroundOrigin( real x, real y, real z, real angle );
   virtual void rotateAroundOrigin( const Vec3& axis, real angle );
   virtual void rotateAroundOrigin( real xangle, real yangle, real zangle );
   virtual void rotateAroundOrigin( const Vec3& euler );

   virtual void rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle );
   virtual void rotateAroundPoint( const Vec3& point, const Vec3& euler );
   //@}
   //**********************************************************************************************

   //**Setup functions*****************************************************************************
   /*!\name Setup functions */
   //@{
   void setSection( Bodies& bodies );
   void clearSection();
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void update( const Vec3& dp );  // Translation update of a subordinate section
   virtual void update( const Quat& dq );  // Rotation update of a subordinate section
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void calcCenterOfMass();  // Calculation of the center of mass
   void calcInertia();       // Calculation of the moment of inertia
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Bodies bodies_;  //!< Rigid bodies contained in this section
                    /*!< The section registers itself with all contained rigid bodies as
                         superordinate body. With this, all forces acting on the subordinate
                         bodies are relayed to the section. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Section operators */
//@{
std::ostream& operator<<( std::ostream& os, const Section& u );
//@}
//*************************************************************************************************

} // namespace pe

#endif
