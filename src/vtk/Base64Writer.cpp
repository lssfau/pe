//=================================================================================================
/*!
 *  \file src/vtk/Base64Writer.cpp
 *  \brief A base64 encoder for the vtk output (ported with curtsey of Jan Goetz)
 *
 *  Copyright (C) 2012 Simon Bogner
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

#include <pe/vtk/Base64Writer.h>


namespace pe {

namespace vtk {

	const char Base64Writer::cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";


   void Base64Writer::encodeblock( unsigned char inBuf[3], unsigned char outBuf[4], int len )
   {
       outBuf[0] = cb64[ inBuf[0] >> 2 ];
       outBuf[1] = cb64[ ((inBuf[0] & 0x03) << 4) | ((inBuf[1] & 0xf0) >> 4) ];
       outBuf[2] = (unsigned char) (len > 1 ? cb64[ ((inBuf[1] & 0x0f) << 2) | ((inBuf[2] & 0xc0) >> 6) ] : '=');
       outBuf[3] = (unsigned char) (len > 2 ? cb64[ inBuf[2] & 0x3f ] : '=');
   }

	Base64Writer::Base64Writer( std::ostream& o )
   : out( o ), buffer( std::ios_base::out | std::ios_base::binary ) {
	}

   void Base64Writer::flush() {
	   // out << buffer.str();
	   unsigned char input[3] = { 0, 0, 0 };
	   unsigned char output[4] = { 0, 0, 0, 0 };
	   std::string str = buffer.str();

	   for ( unsigned int i = 0; i < str.size(); i += 3 ) {
		   if ( str.size() - i < 3 ) {
			   int length = int(str.size()) - i;
			   for ( int j = 0; j < length; j++ )
				   input[j] = str[i+j];
			   encodeblock( input, output, length );
			   out << output[0] << output[1] << output[2] << output[3];
		   }
		   else {
			   input[0] = str[i];
			   input[1] = str[i+1];
			   input[2] = str[i+2];
			   encodeblock( input, output, 3 );
			   out << output[0] << output[1] << output[2] << output[3];
		   }
	   }

	   buffer.str("");
   }

	Base64Writer::~Base64Writer() {
		// TODO Auto-generated destructor stub
	}

} // namespace vtk

} // namespace pe
