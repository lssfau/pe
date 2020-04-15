//=================================================================================================
/*!
 *  \file pe/core/MeshLoader.h
 *  \brief Header file for the mesh loader interface
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

#ifndef _PE_CORE_MESHLOADER_H_
#define _PE_CORE_MESHLOADER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/Normals.h>
#include <pe/core/rigidbody/Vertices.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO.
 * \ingroup core
 *
 * TODO
 */
class MeshLoader
{
public:
   //**Constructors********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual inline ~MeshLoader();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool canLoad( const char* file ) = 0;
   virtual void load( const char* file, Vertices& vertices, Normals& normals ) = 0;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the MeshLoader class.
 */
inline MeshLoader::~MeshLoader()
{}
//*************************************************************************************************

} // namespace pe

#endif
