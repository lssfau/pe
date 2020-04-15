//=================================================================================================
/*!
 *  \file pe/vtk/Base64Writer.h
 *  \brief A base64 encoder for the vtk output (ported from walberla with curtsey of Jan Goetz)
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

#ifndef _PE_BASE64WRITER_H_
#define _PE_BASE64WRITER_H_

#include <iostream>
#include <sstream>


namespace pe {
namespace vtk {
      //*************************************************************************************************
      /*! TODO
       *  \ingroup vtk
       *
       */
      //*************************************************************************************************
	class Base64Writer {
		public:
	      Base64Writer( std::ostream& o );
	      template <class T> Base64Writer& operator<<( const T& data );
	      Base64Writer& operator<<( const char* data);
	      void flush();
			~Base64Writer();

      private:
	      std::ostream& out;
	      std::stringstream buffer;
	      static const char cb64[];
	      static void encodeblock( unsigned char inBuf[3], unsigned char outBuf[4], int len );
	};



   template <class T>
   Base64Writer& Base64Writer::operator<<( const T& data ) {
	   T value = data;
	   buffer.write( reinterpret_cast<char*>( &value ), sizeof( T ) );
	   return *this;
   }


}
}

#endif /* PARAVIEWBASE64WRITER_H_ */
