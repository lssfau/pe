//=================================================================================================
/*!
 *  \file src/povray/Color.cpp
 *  \brief Implementation of a rgb color for the POV-Ray visualization
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
#include <iomanip>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <pe/povray/Color.h>
#include <pe/util/Random.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the Color class.
 *
 * The default color is a solid white.
 */
Color::Color()
   : color_( "rgb <1,1,1>" )  // POV-Ray string representation
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief \a real value constructor for the Color class.
 *
 * \param red The red channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param green The green channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param blue The blue channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param alpha The transparency of the color. Has to be in the range \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid color value.
 *
 * Sets the color channels of the Color object to (\a red, \a green, \a blue) and the transparency
 * of the color to \a alpha. The given values have to be in the range \f$ [0..1] \f$, otherwise a
 * \a std::invalid_argument exception is thrown. The default for the transparency value is 0,
 * which results in a solid, non-transparent color.
 */
Color::Color( real red, real green, real blue, real alpha )
   : color_()
{
   if( red  < 0.0 || red  > 1.0 || green < 0.0 || green > 1.0 ||
       blue < 0.0 || blue > 1.0 || alpha < 0.0 || alpha > 1.0 )
      throw std::invalid_argument( "Invalid color value!" );

   std::ostringstream oss;
   oss << "rgbf <" << red << "," << green << "," << blue << "," << alpha << ">";
   oss.str().swap( color_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Parameter string constructor for the Color class.
 *
 * \param params The input string containing the color parameters.
 * \exception std::invalid_argument Invalid color input string.
 *
 * For details about the format of the color parameter string, see the details of the
 * Color class description.
 */
Color::Color( const std::string& params )
   : color_( "rgb <1,1,1>" )
{
   std::istringstream iss( params );

   if( !parse( iss ) )
      throw std::invalid_argument( "Invalid color input string" );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Changing the color with \a real parameters.
 *
 * \param red The red channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param green The green channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param blue The blue channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param alpha The transparency of the color. Has to be in the range \f$ [0..1] \f$.
 * \return void
 * \exception std::invalid_argument Invalid color value.
 */
void Color::set( real red, real green, real blue, real alpha )
{
   if( red  < 0.0 || red  > 1.0 || green < 0.0 || green > 1.0 ||
       blue < 0.0 || blue > 1.0 || alpha < 0.0 || alpha > 1.0 )
      throw std::invalid_argument( "Invalid color value!" );

   std::ostringstream oss;
   oss << "rgbf <" << red << "," << green << "," << blue << "," << alpha << ">";
   oss.str().swap( color_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the color with a color parameters string.
 *
 * \param params The input string containing the color parameters.
 * \return void
 * \exception std::invalid_argument Invalid color input string.
 *
 * For details about the format of the color parameter string, see the details of the
 * Color class description.
 */
void Color::set( const std::string& params )
{
   std::istringstream iss( params );

   if( !parse( iss ) )
      throw std::invalid_argument( "Invalid color< input string" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray color.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Color::print( std::ostream& os, bool newline ) const
{
   os << "color " << color_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray color.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the color output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Color::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "color " << color_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the color by parsing the input string
 *
 * \param is Reference to the input stream containing the color parameters.
 * \return \a true if the color was successfully extracted, \a false if not.
 *
 * The Parse function extracts the color parameters from the given input stream. In case of an
 * input error, the Color object is not changed, the \a std::istream::failbit is set and the
 * function returns \a false.
 */
bool Color::parse( std::istream& is )
{
   Color::String key;

   // Extracting the keyword
   if( !(is >> key) || ( key.compare( "color" ) == 0 && !(is >> key) ) ) {
      is.setstate( std::istream::failbit );
      return false;
   }

   // Extracting a color in the format 'rgb <red,green,blue>'
   if( key.compare( "rgb" ) == 0 )
   {
      real red(0.0), green(0.0), blue(0.0);
      char bracket1, bracket2, comma1, comma2;

      if( !(is >> bracket1 >> red >> comma1 >> green >> comma2 >> blue >> bracket2) ||
          bracket1 != '<' || comma1 != ',' || comma2 != ',' || bracket2 != '>' ||
          red < 0.0 || red > 1.0 || green < 0.0 || green > 1.0 || blue < 0.0 || blue > 1.0 )
      {
         is.setstate( std::istream::failbit );
         return false;
      }

      std::ostringstream oss;
      oss << "rgb <" << red << "," << green << "," << blue << ">";
      oss.str().swap( color_ );
   }

   // Extracting a color in the format 'rgbf <red,green,blue,alpha>'
   else if( key.compare( "rgbf" ) == 0 )
   {
      real red(0.0), green(0.0), blue(0.0), alpha(0.0);
      char bracket1, bracket2, comma1, comma2, comma3;

      if( !(is >> bracket1 >> red >> comma1 >> green >> comma2 >> blue >> comma3 >> alpha >> bracket2) ||
          bracket1 != '<' || comma1 != ',' || comma2 != ',' || bracket2 != '>' || red < 0.0 || red > 1.0 ||
          green < 0.0 || green > 1.0 || blue < 0.0 || blue > 1.0 || alpha < 0.0 || alpha > 1.0 )
      {
         is.setstate( std::istream::failbit );
         return false;
      }

      std::ostringstream oss;
      oss << "rgbf <" << red << "," << green << "," << blue << "," << alpha << ">";
      oss.str().swap( color_ );
   }

   // Extracting a random color
   else if( key.compare( "random" ) == 0 )
   {
      // Creating a random rgb color
      std::ostringstream oss;
      oss << "rgb <" << rand<real>() << "," << rand<real>() << "," << rand<real>() << ">";
      oss.str().swap( color_ );
   }

   // Extracting a predefined POV-Ray color
   else color_ = key.str();

   return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Color class.
 * \ingroup povray_color
 *
 * \param os Reference to the output stream.
 * \param color Reference to a color object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Color& color )
{
   color.print( os, false );
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global input operator for the Color class.
 * \ingroup povray_color
 *
 * \param is Reference to the input stream.
 * \param color Reference to a color object.
 * \return The input stream.
 *
 * For details about the format of the color parameters, see the details of the Color class
 * description. The input operator guarantees that the color object is not changed in the
 * case of an input error. In the case of an invalid input, the stream is return to its
 * previous position and the \a std::istream::failbit is set.
 */
std::istream& operator>>( std::istream& is, Color& color )
{
   if( !is ) return is;

   const std::istream::pos_type pos( is.tellg() );
   const std::istream::fmtflags oldFlags( is.flags() );

   // Setting the 'skip whitespaces' flag
   is >> std::skipws;

   // Extracting the color
   if( !color.parse( is ) ) {
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
/*!\brief Global input operator for the Color::String class.
 * \ingroup povray_color
 *
 * \param is Reference to the input stream.
 * \param s Reference to a string object.
 * \return The input stream.
 *
 * The input operator extracts the next string consisting of alphanumeric character from the
 * stream until the next non-alphanumeric character is encountered (exception: the '_' character
 * is counted as alphanumeric character).
 */
std::istream& operator>>( std::istream& is, Color::String& s )
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
