//=================================================================================================
/*!
 *  \file src/povray/ColorMap.cpp
 *  \brief Implementation of a color map for the POV-Ray visualization
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

#include <cctype>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <pe/povray/Color.h>
#include <pe/povray/ColorMap.h>
#include <pe/povray/RandomColor.h>
#include <pe/system/Precision.h>
#include <pe/util/Random.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the ColorMap class.
 *
 * The default color map appearance is a blend from Black to White.
 */
ColorMap::ColorMap()
   : colormap_( "   [0.0 color rgb <0,0,0>]\n"
                "   [1.0 color rgb <1,1,1>]\n" )  // POV-Ray string representation
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Parameter string constructor for the ColorMap class.
 *
 * \param params The input string containing the color map parameters.
 * \exception std::invalid_argument Invalid color map input string.
 *
 * For details about the format of the color map parameter string, see the details of the
 * ColorMap class description.
 */
ColorMap::ColorMap( const std::string& params )
   : colormap_( "   [0.0 color Black]\n   [1.0 color White]\n" )  // POV-Ray string representation
{
   std::istringstream iss( params );

   if( !parse( iss ) )
      throw std::invalid_argument( "Invalid color map input string" );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the color map with a parameter string.
 *
 * \param params The input string containing the color map parameters.
 * \return void
 * \exception std::invalid_argument Invalid color map input string.
 *
 * For details about the format of the color map parameter string, see the details of the
 * ColorMap class description.
 */
void ColorMap::set( const std::string& params )
{
   std::istringstream iss( params );

   if( !parse( iss ) )
      throw std::invalid_argument( "Invalid color map input string" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray color map.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void ColorMap::print( std::ostream& os, bool newline ) const
{
   os << "color_map {\n" << colormap_ << "}";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray color map.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the color map output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void ColorMap::print( std::ostream& os, const char* tab, bool newline ) const
{
   std::string line;
   std::istringstream iss( colormap_ );

   os << tab << "color_map {\n";

   while( std::getline( iss, line ) ) {
      os << tab << line << "\n";
   }

   os << tab << "}";

   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the color map by parsing the input string
 *
 * \param is Reference to the input stream containing the color map parameters.
 * \return \a true if the color map was successfully extracted, \a false if not.
 *
 * The parse function extracts the color map parameters from the given input stream. In case of
 * an input error, the ColorMap object is not changed, the \a std::istream::failbit is set and
 * the function returns \a false.
 */
bool ColorMap::parse( std::istream& is )
{
   char c;
   unsigned int counter(0);
   real newValue(0.0), oldValue(0.0);
   Color color;
   std::ostringstream oss;

   // Parsing a color map parameter string
   if( is >> std::ws && !is.eof() && ( c = is.peek() ) == '[' )
   {
      do {
         if( !(is >> c >> newValue >> color >> c) || c != ']' ||
            newValue < 0.0 || newValue > 1.0 || newValue < oldValue )
         {
            is.setstate( std::istream::failbit );
            return false;
         }
         else {
            ++counter;
            oldValue = newValue;
            oss << "   [" << newValue << " " << color << "]\n";
         }
      }
      while( is >> std::ws && !is.eof() && ( c = is.peek() ) == '[' );

      // Checking the size of the color map
      if( counter < 2 || counter > 20 ) {
         is.setstate( std::istream::failbit );
         return false;
      }
   }

   // Extracting a random or predefined POV-Ray color map
   else
   {
      ColorMap::String identifier;

      // Extracting the identifier
      if( !(is >> identifier) )
      {
         is.setstate( std::istream::failbit );
         return false;
      }

      // Creating a random color map
      else if( identifier.compare( "random" ) == 0 )
      {
         const unsigned int n( rand<unsigned int>( 2, 5 ) );
         real threshold( 0 );

         for( unsigned int i=0; i<n; ++i ) {
            threshold = rand<real>( threshold, 1 );
            oss << "   [" << threshold << " " << RandomColor() << "]\n";
         }
      }

      // Creating a predefined color map
      else oss << "   " << identifier.str() << "\n";
   }

   // Finalizing and committing the color map
   oss.str().swap( colormap_ );
   return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the ColorMap class.
 * \ingroup povray_colormap
 *
 * \param os Reference to the output stream.
 * \param colormap Reference to a color map object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const ColorMap& colormap )
{
   colormap.print( os, false );
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global input operator for the ColorMap class.
 * \ingroup povray_colormap
 *
 * \param is Reference to the input stream.
 * \param colormap Reference to a color map object.
 * \return The input stream.
 *
 * For details about the format of the color map parameters, see the details of the ColorMap
 * class description. The input operator guarantees that the color map object is not changed
 * in the case of an input error. In the case of an invalid input, the stream is return to
 * its previous position and the \a std::istream::failbit is set.
 */
std::istream& operator>>( std::istream& is, ColorMap& colormap )
{
   if( !is ) return is;

   const std::istream::pos_type pos( is.tellg() );
   const std::istream::fmtflags oldFlags( is.flags() );

   // Setting the 'skip whitespaces' flag
   is >> std::skipws;

   // Extracting the color map
   if( !colormap.parse( is ) ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   // Resetting the flags
   is.flags( oldFlags );

   return is;
}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Global input operator for the ColorMap::String class.
 * \ingroup povray_colormap
 *
 * \param is Reference to the input stream.
 * \param s Reference to a string object.
 * \return The input stream.
 *
 * The input operator extracts the next string consisting of alphanumeric character from the
 * stream until the next non-alphanumeric character is encountered (exception: the '_' character
 * is counted as alphanumeric character).
 */
std::istream& operator>>( std::istream& is, ColorMap::String& s )
{
   if( !is ) return is;

   char c;

   // Skipping any leading whitespaces
   is >> std::ws;

   // Extracting the string
   s.str_.clear();
   while( is ) {
      c = is.peek();
      if( std::isalnum( c ) || c == '_' ) {
         s.str_ += c;
         is.ignore();
      }
      else break;
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************

} // namespace povray

} // namespace pe
