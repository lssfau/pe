//=================================================================================================
/*!
 *  \file src/core/OBJMeshLoader.cpp
 *  \brief Source file for an OBJ parameter file extractor
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


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pe/util.h>
#include <pe/core/OBJMeshLoader.h>



namespace pe {

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief checks if the file extension is ".obj" and therefore she be appropriate to parse.
 *
 * \param file Path or filename of the file that should be checked
 */
bool OBJMeshLoader::canLoad( const std::string file )
{
   return file.find(".obj") != std::string::npos || file.find(".OBJ") != std::string::npos;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param file The OBJ parameter file.
 * \param vertices TODO
 * \param faceIndices TODO
 * \param normals TODO
 * \param normalIndices TODO
 * \param textCoordinates TODO
 * \param textIndices TODO
 * \param clockwise points on a face are ordered clockwise. Default is \a false.
 * \param lefthanded the coordiantes are given vor a left handed coordinate system. Default is \a false.
 * \return void
 * \exception std::runtime_error Invalid/corrupt STL file.
 */
void OBJMeshLoader::load( const std::string file,
                           Vertices& vertices, IndicesLists& faceIndices,
                           Normals& normals, IndicesLists& normalIndices,
                           TextureCoordinates& textCoordinates, IndicesLists& textIndices,
                           bool clockwise, bool lefthanded)
{
   std::ifstream objFile(file.c_str());

   if(objFile.fail()) {
      std::string error = "OBJFileReader: Failed to open "+ file;
      throw std::runtime_error(error);
   }

   std::string token;
   std::string line;
   real param1 = 0.0;
   real param2 = 0.0;
   real param3 = 0.0;
   real param4 = 0.0;

   real lefthandCorrect = lefthanded ? real(-1.0) : real(1.0);

   size_t lineCount (0);

   int groups = 0;

   while(getline(objFile, line)) {
      ++lineCount;
      //ignore empty lines
      if (line.size() == 0) {
         continue;
      }

      std::istringstream lineSS(line);
      lineSS >> token;

      //Reading vertex
      if(0 == token.compare("v")) {
         lineSS >> param1 >> param2 >> param3 >> param4;
         Vec3 vertex(param1, param2, lefthandCorrect*param3);
         manageCapacity(vertices);
         vertices.push_back(vertex);
         if(param4 != real( 0 )) {
            std::stringstream error;
            error << "OBJFileReader: Fourth parameter of normal (w) is optional and therefore ignored."
                     << " (" << file << ":" <<  lineCount <<")\n";
            pe_LOG_DETAIL_SECTION(log) { //TODO DETAIL SECTION richtig?
               log << error.str();
            }
         }
      }
      //Reading texture coordinate
      else if(0 == token.compare("vt")) {
         lineSS >> param1 >> param2 >>param4;
         Vec2 coord(param1, param2);
         manageCapacity(textCoordinates);
         textCoordinates.push_back(coord);
         if(param4 != real( 0 )) {
            std::stringstream error;
            error << "OBJFileReader: Fourth parameter of texture coordinate (w) is optional and therefore ignored."
                     << " (" << file << ":" <<  lineCount <<")\n";
            pe_LOG_DETAIL_SECTION(log) {
               log << error.str();
            }
         }
      }
      //Reading vertex normal
      else if(0 == token.compare("vn")) {
         lineSS >> param1 >> param2 >> param3;
         Vec3 normal(param1, param2, lefthandCorrect*param3);
         manageCapacity(normals);
         normals.push_back(normal);
      }
      //Ignoring # comments
      else if(0 == token.compare("#") || (token.length()>0 && '#'==token[0])) {
         continue;
      }
      //groups are ignored!
      //All groups are considered to describe one single object
      else if(0 == token.compare("g")) {
         if (groups > 1) {
            std::stringstream error;
            error << "OBJFileReader: Groups are ignored!" << " (" << file << ":" <<  lineCount <<")\n"
                     <<"All groups are considered to describe one single object.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
         }
         ++groups;
         continue;
      }
      //Reading face properties
      else if(0 == token.compare("f")) {
         Vector3<size_t> vertexIndex(0); //0 is not a valid index for OBJ-Files as they start counting at 1
         Vector3<size_t> texturecoord(0);
         Vector3<size_t> normalIndex(0);
         for(int i = 0; i< 3; i++) {
            token = "";
            lineSS >> token;
            std::istringstream iss(token);

            //read vertex index
            if(getline(iss, token, '/')) {
               std::stringstream ssv(token);
               ssv >> vertexIndex[i];
               /* fancy negative indices for backward referencing not supported
               if (vertexIndex[i] < 0) {
                  vertexIndex[i] = vertices.size() - vertexIndex[i] +1; //+1 weil 1-baesed
               }*/
               if (vertexIndex[i] > vertices.size()) {
                  std::stringstream error;
                  error << "OBJFileReader: vertex index out of bound. "<< vertexIndex[i] << " > "<< vertices.size()<< " (" << file << ":" <<  lineCount <<").\n";
                  pe_LOG_INFO_SECTION(log) {
                     log << error.str();
                  }
                  throw std::out_of_range(error.str());
               }
            }
            else {
               std::stringstream error;
               error << "OBJFileReader: Missing face data in line '"<< line << "' (" << file << ":" <<  lineCount <<").\n";
               pe_LOG_INFO_SECTION(log) {
                  log << error.str();
               }
               throw std::runtime_error(error.str());
            }

            //read texture coordinate index
            if(getline(iss, token, '/')) {
               std::stringstream sst(token);
               sst >> texturecoord[i];
               /*fancy negative indices for backward referencing not supported
               if (texturecoord[i] < 0) {
                  texturecoord[i] = textCoordinates.size() - texturecoord[i] +1; //+1 weil 1-baesed
               }*/
               if (texturecoord[i] > textCoordinates.size()) {
                  std::stringstream error;
                  error << "OBJFileReader: texture coordinate index out of bound. "<< texturecoord[i] << " > "<< textCoordinates.size()<< " (" << file << ":" <<  lineCount <<").\n";
                  pe_LOG_INFO_SECTION(log) {
                     log << error.str();
                  }
                  throw std::out_of_range(error.str());
               }

               //read normal index
               if(getline(iss, token, '/')) {
                  std::stringstream ssn(token);
                  ssn >> normalIndex[i];
                  /* fancy negative indices for backward referencing not supported
                  if (normalIndex[i] < 0) {
                     normalIndex[i] = normals.size() - normalIndex[i] +1; //+1 weil 1-baesed
                  }*/
                  if (normalIndex[i] > normals.size()) {
                     std::stringstream error;
                     error << "OBJFileReader: normal index out of bound. "<< normalIndex[i] << " > "<< normals.size()<< " (" << file << ":" <<  lineCount <<").\n";
                     pe_LOG_INFO_SECTION(log) {
                        log << error.str();
                     }
                     throw std::out_of_range(error.str());
                  }
               }
               else {
                  //no normal index is given, this is not a problem!
                  //http://en.wikipedia.org/wiki/Wavefront_.obj_file#Vertex.2Ftexture-coordinate
                  /*
                  std::stringstream error;
                  error << "OBJFileReader: Error while reading face data, missing normal index in line '" << line << "' (" << file << ":" <<  lineCount <<").\n";
                  pe_LOG_INFO_SECTION(log) {
                     log << error.str();
                  }
                  throw std::out_of_range(error.str());
                  */
               }
            }
            else {
               //only vertex index is given
            }

         }
         //check if line is completely parsed
         if(!lineEndOK(lineSS)) {
            std::stringstream error;
            error << "OBJFileReader: unexpected end of line in '" << line <<"' (" << file << ":" <<  lineCount <<").\n";
            error << "Only triangular faces are currently supported.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
            throw std::runtime_error(error.str());
         }


         //vector for correcting the index form 1-based to 0-based counting
         const Vector3<size_t> oneVec(1,1,1);

         //save the face indeces
         manageCapacity(faceIndices);
         if(clockwise) {
            std::swap(vertexIndex[1], vertexIndex[2]);
         }
         faceIndices.push_back(vertexIndex-oneVec);

         //save the textureIndices if given
         if(texturecoord[0] && texturecoord[1] && texturecoord[2]) {
            manageCapacity(textIndices);
            if(clockwise) {
               std::swap(texturecoord[1], texturecoord[2]);
            }
            textIndices.push_back(texturecoord-oneVec);
         }

         //save the textureIndices if given
         if(normalIndex[0] && normalIndex[1] && normalIndex[2]) {
            manageCapacity(normalIndices);
            if(clockwise) {
               std::swap(normalIndex[1], normalIndex[2]);
            }
            normalIndices.push_back(normalIndex-oneVec);
         }

      }
      //ignoring unssuportet options
      //http://www.fileformat.info/format/wavefrontobj/egff.htm
      else if(   0 == token.compare("vp")    //Parameter space vertices
              || 0 == token.compare("deg")   //Degree
              || 0 == token.compare("bmat")  //Basis matrix
              || 0 == token.compare("step")  //Step size
              || 0 == token.compare("p")     //Point
              || 0 == token.compare("l")     //Line
              || 0 == token.compare("curv")  //Curve
              || 0 == token.compare("curv2") //2D curve
              || 0 == token.compare("surf")  //Surface
              || 0 == token.compare("parm")  //Parameter values
              || 0 == token.compare("trim")  //Outer trimming loop
              || 0 == token.compare("hole")  //Inner trimming loop
              || 0 == token.compare("scrv")  //Special curve
              || 0 == token.compare("sp")    //Special point
              || 0 == token.compare("end")   //End statement
              || 0 == token.compare("con")   //Connect
              || 0 == token.compare("s")     //Smoothing group
              || 0 == token.compare("mg")    //Merging group
              || 0 == token.compare("o")     //Object name
              || 0 == token.compare("bevel") //Bevel interpolation
              || 0 == token.compare("c_interp") //Color interpolation
              || 0 == token.compare("d_interp") //Dissolve interpolation
              || 0 == token.compare("lod")   //Level of detail
              || 0 == token.compare("usemtl")//Material name
              || 0 == token.compare("mtllib")//Material library
              || 0 == token.compare("shadow_obj")//Shadow casting
              || 0 == token.compare("trace_obj") //Ray tracing
              || 0 == token.compare("ctech") //Curve approximation technique
              || 0 == token.compare("stech") //Surface approximation technique
                ) {
         std::stringstream error;
         error << "OBJFileReader: ignoring line  '" << line << "' beginning with unsupported but valid token '" << token  << "'. (" << file << ":" <<  lineCount <<")\n";
         pe_LOG_DETAIL_SECTION(log) {
            log << error.str();
         }
         continue;
      }
      else {
         std::stringstream error;
         error << "OBJFileReader: ignoring line  '" << line << "' beginning with unknown token '" << token  << "'. (" << file << ":" <<  lineCount <<")\n";
         pe_LOG_INFO_SECTION(log) {
            log << error.str();
         }
         continue;
      }

      //check if line is completely parsed
      if(!lineEndOK(lineSS)) {
         std::stringstream error;
         error << "OBJFileReader: unexpected token in line:  '" << line << "'. (" << file << ":" <<  lineCount <<")\n";
         pe_LOG_INFO_SECTION(log) {
            log << error.str();
         }
      }
   }

   if(!objFile.good() && !objFile.eof()) {
      std::stringstream error;
      error << "OBJFileReader: Error while reading "<< file << "\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::runtime_error(error.str());
   }

   //check if vertices are given
   if(vertices.size() == 0 ){
      std::stringstream error;
      error << "OBJFileReader: No vertices described in file " << file << ".\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::runtime_error(error.str());
   }

   //check if faces are given
   if(faceIndices.size() < 4 ){
      std::stringstream error;
      error << "OBJFileReader: Less than 4 faces described in file " << file << ".\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::runtime_error(error.str());
   }

   //check for correct number of normals indices
   if(normalIndices.size() != 0 && normalIndices.size() != faceIndices.size()){
      std::stringstream error;
      error << "OBJFileReader: Number of normals indices differs with number of faces indices.\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::runtime_error(error.str());
   }

   //check for correct number of texture coordinate indices
   if(textIndices.size() != 0 && textIndices.size() != faceIndices.size()){
      std::stringstream error;
      error << "OBJFileReader: Number of texture coordinate indices differs with number of faces indices.\n";
error << "#text=" << textIndices.size() << " #faces=" << faceIndices.size() << std::endl;
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::runtime_error(error.str());
   }

   //TODO if c++11 is available then evoke shrink_to_fit() on all vectors

}

bool OBJMeshLoader::lineEndOK(std::istringstream& lineSS) {
   std::string token;
   lineSS >> token;
   if(token.size() != 0 &&  !(0 == token.compare("#") ||  '#'==token[0]) ) {
      return false;
   }
   return true;
}

}
//*************************************************************************************************
