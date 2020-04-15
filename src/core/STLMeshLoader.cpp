//=================================================================================================
/*!
 *  \file src/core/STLMeshLoader.cpp
 *  \brief Source file for an STL parameter file extractor
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

#include <cstring>
#include <fstream>
#include <stdexcept>
#include <pe/core/STLMeshLoader.h>
#include <pe/util/Constraints.h>


namespace pe {

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO.
 *
 * \param file TODO
 */
bool STLMeshLoader::canLoad( const char* file )
{
   return std::strstr( file, ".stl" ) != 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO.
 *
 * \param file The STL parameter file.
 * \param vertices TODO.
 * \param normals TODO.
 * \return void
 * \exception std::runtime_error Invalid/corrupt STL file.
 */
void STLMeshLoader::load( const char* file, Vertices& vertices, Normals& normals )
{
   char tmp[5];
   std::ifstream in( file, std::ifstream::in | std::ifstream::binary );

   if( !in.is_open() || !in.read( tmp, 5 ) )
      throw std::runtime_error( "Invalid STL file" );

   if( std::strncmp( tmp, "solid", 5 ) == 0 ) {
      in.close();
      in.open( file, std::ifstream::in );
      readASCIIFile( in, vertices, normals );
   }
   else {
      readBinaryFile( in, vertices, normals );
   }

   // Closing the file stream is handled via its destructor.
}
//*************************************************************************************************




//=================================================================================================
//
//  INPUT AND OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO.
 *
 * \param in TODO.
 * \param vertices TODO.
 * \param normals TODO.
 * \return void
 * \exception std::runtime_error Corrupt STL file.
 */
void STLMeshLoader::readASCIIFile( std::ifstream& in, Vertices& vertices, Normals& normals ) const
{
   std::string word, name;
   Normal n;
   Vertex a, b, c;

   word.reserve( 100 );
   name.reserve( 100 );

   // Extracting the head line including the 'solid' keyword
   if( !std::getline( in, name ) )
      throw std::runtime_error( "Corrupt STL file: invalid header line" );

   // Extracting the facets of the triangle mesh
   while( in >> word )
   {
      // Extracting a triangle facet
      if( word.compare( "facet" ) == 0 )
      {
         // Extracting the normal
         if( !(in >> n) ) {
            throw std::runtime_error( "Corrupt STL file: invalid facet normal" );
         }
         else {
            normals.push_back( n );
         }

         // Extracting the 'outer loop' keywords
         if( !(in >> word) || word.compare( "outer" ) != 0 || !(in >> word) || word.compare( "loop" ) != 0 ) {
            throw std::runtime_error( "Corrupt STL file: invalid 'outer loop' keywords" );
         }

         // Extracting the three triangle vertices
         // The vertices are added to the vertex vector in counter-clockwise (right-handed) order.
         if( !(in >> a >> b >> c ) ) {
            throw std::runtime_error( "Corrupt STL file: invalid vertex" );
         }
         else if( trans( (b-a) % (c-a) ) * n < real(0) ) {
            vertices.push_back( a );
            vertices.push_back( c );
            vertices.push_back( b );
         }
         else {
            vertices.push_back( a );
            vertices.push_back( b );
            vertices.push_back( c );
         }

         // Extracting the 'endloop' keyword
         if( !(in >> word) || word.compare( "endloop" )  != 0 ) {
            throw std::runtime_error( "Corrupt STL file: missing 'endloop' keyword" );
         }

         // Extracting the 'endfacet' keyword
         if( !(in >> word) || word.compare( "endfacet" )  != 0 ) {
            throw std::runtime_error( "Corrupt STL file: missing 'endfacet' keyword" );
         }
      }

      // Ending the STL mesh extraction
      else if( word.compare( "endsolid" ) == 0 ) {
         return;
      }

      // Treatment of unknown keywords
      else {
         throw std::runtime_error( "Corrupt STL file: unknown keyword encountered" );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO.
 *
 * \param in TODO.
 * \param vertices TODO.
 * \param normals TODO.
 * \return void
 * \exception std::runtime_error Corrupt STL file.
 */
void STLMeshLoader::readBinaryFile( std::ifstream& in, Vertices& vertices, Normals& normals ) const
{
   // The following constraint makes sure the unsigned int type is 4 bytes large.
   // This size is used for the number of facets in the STL file.
   pe_CONSTRAINT_MUST_HAVE_SIZE( unsigned int, 4 );

   char header[75];
   unsigned int num( 0 );
   BinaryVector n, a, b, c;

   // Extracing the remaining 75 bytes of the 80 byte header
   in.read( header, 75 );

   // Extracting the number of facets
   in.read( reinterpret_cast<char*>( &num ), sizeof( unsigned int ) );

   // Extracting the facets
   // The vertices are added to the vertex vector in counter-clockwise (right-handed) order.
   for( unsigned int i=0; i<num; ++i )
   {
      if( !(in >> n >> a >> b >> c) ) {
         throw std::runtime_error( "Corrupt STL file: input error during facet extraction" );
      }
      else if( trans( (b-a) % (c-a) ) * n < real(0) ) {
         normals.push_back( n );
         vertices.push_back( a );
         vertices.push_back( c );
         vertices.push_back( b );
      }
      else {
         normals.push_back( n );
         vertices.push_back( a );
         vertices.push_back( b );
         vertices.push_back( c );
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS STLMESHLOADER::VERTEX
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the STLMeshLoader::Vertex class.
 *
 * TODO
 */
std::istream& operator>>( std::istream& is, STLMeshLoader::Vertex& v )
{
   if( !is ) return is;

   std::string tmp;
   real x, y, z;
   const std::istream::pos_type pos( is.tellg() );

   // Extracting the ASCII vector data
   if( !(is >> tmp >> x >> z >> y) || tmp.compare( "vertex" ) != 0 ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   // Transfering the input to the vector values
   else {
      v[0] = x; v[1] = y; v[2] = z;
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS STLMESHLOADER::NORMAL
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the STLMeshLoader::Normal class.
 *
 * TODO
 */
std::istream& operator>>( std::istream& is, STLMeshLoader::Normal& v )
{
   if( !is ) return is;

   std::string tmp;
   real x, y, z;
   const std::istream::pos_type pos( is.tellg() );

   // Extracting the ASCII vector data
   if( !(is >> tmp >> x >> z >> y) || tmp.compare( "normal" ) != 0 ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   // Transfering the input to the vector values
   else {
      v[0] = x; v[1] = y; v[2] = z;
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS STLMESHLOADER::BINARYVECTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the STLMeshLoader::BinaryVector class.
 *
 * TODO
 */
std::istream& operator>>( std::istream& is, STLMeshLoader::BinaryVector& v )
{
   // The following constraint makes sure the float type is 4 bytes large. This size is
   // occupied by every floating point value in a binary STL file.
   pe_CONSTRAINT_MUST_HAVE_SIZE( float, 4 );

   if( !is ) return is;

   float x, y, z;
   const std::istream::pos_type pos( is.tellg() );

   // Extracting the binary vector data
   if( !is.read( reinterpret_cast<char*>( &x ), sizeof( float ) ) ||
       !is.read( reinterpret_cast<char*>( &z ), sizeof( float ) ) ||
       !is.read( reinterpret_cast<char*>( &y ), sizeof( float ) ) ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   // Transfering the input to the vector values
   else {
      v[0] = x; v[1] = y; v[2] = z;
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe
