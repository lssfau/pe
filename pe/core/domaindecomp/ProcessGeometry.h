//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/ProcessGeometry.h
 *  \brief Header file for the ProcessGeometry base class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_DOMAINDECOMP_PROCESSGEOMETRY_H_
#define _PE_CORE_DOMAINDECOMP_PROCESSGEOMETRY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <list>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>
#include <utility>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for all remote MPI process geometries.
 * \ingroup domaindecomp
 *
 * The ProcessGeometry class represents the geometry, i.e., the physical expansion, of a
 * remote MPI process. The local process is connected to a remote process by defining the
 * geometry of the remote process such that the local process can evaluate if rigid bodies
 * have to be sent to the remote process.\n
 * The ProcessGeometry class works as a base class for all process geometries. It merely
 * defines the interface for all derived classes according to the Protocol pattern. Each
 * derived class must implement the functions to answer the questions whether a rigid body
 * intersects with the remote process geometry and if a global coordinate is contained in the
 * remote process.
 */
struct PE_PUBLIC ProcessGeometry
{
   //**Constructors********************************************************************************
   // No explicitly declared constructor.
   // No explicitly declared copy constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~ProcessGeometry() = 0;
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID     b ) const = 0;
   virtual bool intersectsWith( ConstSphereID   s ) const = 0;
   virtual bool intersectsWith( ConstBoxID      b ) const = 0;
   virtual bool intersectsWith( ConstCapsuleID  c ) const = 0;
   virtual bool intersectsWith( ConstCylinderID c ) const = 0;
   virtual bool intersectsWith( ConstUnionID    u ) const = 0;
   virtual bool containsPoint        ( const Vec3& gpos ) const = 0;
   virtual bool containsPointStrictly( const Vec3& gpos ) const = 0;
   //@}
   //**********************************************************************************************

   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const = 0;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os                  ) const = 0;
   virtual void print( std::ostream& os, const char* tab ) const = 0;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
