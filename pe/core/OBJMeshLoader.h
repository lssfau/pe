//=================================================================================================
/*!
 *  \file pe/core/OBJMeshLoader.h
 *  \brief Header file for an OBJ parameter file extractor
 *
 *  Copyright (C) 2013-2014 Tobias Scharpff
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


#ifndef OBJMESHLOADER_H_
#define OBJMESHLOADER_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************
#include <vector>
#include <string>

#include <pe/core/rigidbody/TriangleMeshTypes.h>

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
//TODO: ventuell sollte das von MeshLoader erben ?
class OBJMeshLoader
{
public:
   //**Constructors********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************


   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual inline ~OBJMeshLoader();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Parsing functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void load(const std::string file,
                     Vertices& vertices, IndicesLists& faceIndices,
                     Normals& normals, IndicesLists& normalIndices,
                     TextureCoordinates& textCoordinates, IndicesLists& textIndices,
                     bool clockwise=false, bool lefthanded=false);
   //@}
   //**********************************************************************************************


   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool canLoad(const std::string file );
   //@}
   //**********************************************************************************************

private:
   //**private Utility functions***************************************************************************
   /*!\name private Utility functions */
   //@{
   virtual bool lineEndOK(std::istringstream& lineSS);
   template <typename T>
   inline void manageCapacity(std::vector<T>& vec);

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
/*!\brief Destructor for the OBJMeshLoader class.
 */
inline OBJMeshLoader::~OBJMeshLoader()
{}
//*************************************************************************************************



template <typename T>
inline void  OBJMeshLoader::manageCapacity(std::vector<T>& vec) {
   if(vec.capacity() == 0) {
      vec.reserve(4);//closed triangle mash consist of at least 4 vertices/faces (tetrahedron)
   }
   else if(vec.capacity() == vec.size()) {
      vec.reserve(2*vec.capacity()); //Exponential resizing of the vector do prevent frequent memory reallocation
   }
}


} // namespace pe


#endif /* OBJMESHLOADER_H_ */
