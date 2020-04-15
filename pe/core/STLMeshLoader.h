//=================================================================================================
/*!
 *  \file pe/core/STLMeshLoader.h
 *  \brief Header file for an STL parameter file extractor
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

#ifndef _PE_CORE_STLMESHLOADER_H_
#define _PE_CORE_STLMESHLOADER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <sstream>
#include <string>
#include <vector>
#include <pe/core/MeshLoader.h>
#include <pe/core/rigidbody/Normals.h>
#include <pe/core/rigidbody/Vertices.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


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
class STLMeshLoader : public MeshLoader
{
public:
   //**Constructors********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual inline ~STLMeshLoader();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool canLoad( const char* file );
   virtual void load( const char* file, Vertices& vertices, Normals& normals );
   //@}
   //**********************************************************************************************

private:
   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   void readASCIIFile( std::ifstream& in, Vertices& vertices, Normals& normals ) const;
   void readBinaryFile( std::ifstream& in, Vertices& vertices, Normals& normals ) const;
   //@}
   //**********************************************************************************************

   //**Private class Vertex************************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief TODO. */
   class Vertex : public Vector3<real> {};
   friend std::istream& operator>>( std::istream& is, Vertex& v );
   /*! \endcond */
   //**********************************************************************************************

   //**Private class Normal************************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief TODO. */
   class Normal : public Vector3<real> {};
   friend std::istream& operator>>( std::istream& is, Normal& v );
   /*! \endcond */
   //**********************************************************************************************

   //**Private class BinaryVector******************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief TODO. */
   class BinaryVector : public Vector3<real> {};
   friend std::istream& operator>>( std::istream& is, BinaryVector& v );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the STLMeshLoader class.
 */
inline STLMeshLoader::~STLMeshLoader()
{}
//*************************************************************************************************

} // namespace pe

#endif
