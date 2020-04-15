//=================================================================================================
/*!
 *  \file pe/povray/Color.h
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

#ifndef _PE_POVRAY_COLOR_H_
#define _PE_POVRAY_COLOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>
#include <pe/system/Precision.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_color Color
 * \ingroup povray
 */
/*!\brief A rgbf color value.
 * \ingroup povray_color
 *
 * The Color class represents a rgbf color value. A color is a composite of the three basic colors
 * red, green and blue and a transparency value (alpha). All four values are values between
 * \f$ [0..1] \f$. A value of 0 indicates the corresponding minimum, a value of 1 indicates the
 * maximum. For example, a solid black color is given by (0,0,0,0), whereas a transparent white
 * is given by (1,1,1,1). There are six different ways to set up a color:

   \code
   pe::Color white( 1.0, 1.0, 1.0, 0.0 );  (1)
   pe::Color black( "rgb <0,0,0>" );       (2)
   pe::Color color;
   color.set( 1.0, 0.0, 0.0 );             (3)
   color.set( "rgb <1,0,0>" );             (4)
   color = pe::povray::RandomColor();      (5)
   is >> color;                            (6)
   \endcode

 * -# Option 1 is the direct setup with pe::real color values. Using this constructor, the red,
 *    green, blue and transparency values can be directly modified with values in the range
 *    \f$ [0..1] \f$. The default value for the transparency is 0.
 * -# Option 2 is the direct setup with a parameter string. The parameter string has to be
 *    contain the color parameters according to the POV-Ray color format (see below).
 * -# Option 3 is similar to the first setup, but can be used e.g. after a default setup of the
 *    color via the default constructor. The red, green, blue and alpha values have to be in
 *    the range \f$ [0..1] \f$. The default value for the transparency is 0.
 * -# Option 4 is the correspondent to option 2 and can be used to change a color. Again, the
 *    parameter string has to contain the color parameters according to the POV-Ray color format
 *    (see below).
 * -# Option 5 demonstrates the setup of a random color with a transparency of 0.
 * -# Option 6 shows the setup of a color via an input stream. The color can be configured from
 *    any input stream containing the color parameters according to the POV-Ray color format
 *    (see below).\n
 *
 * To specify a color with a color parameter string, one of the following formats can be used:

   \code
   rgb <1,1,0>         (1)
   rgbf <1,1,0,0>      (2)
   White/Black/Red     (3)
   random              (4)
   \endcode

 * -# The first format specifies a rgb color consisting of a red, green and blue value. All values
 *    have to be in the range \f$ [0..1] \f$. The transparency is set to 0.
 * -# The second format specifies a rgbf color consisting of a red, green, blue and alpha value.
 *    All values have to be in the range \f$ [0..1] \f$.
 * -# Option three specifies a named color value. This can either be a predefined POV-Ray color (as
 *    for example defined in the \a colors.inc header, like e.g. White or Red) or any self-defined
 *    color value.
 * -# The random color creates a random color with a transparency of 0.\n
 *
 * \b Note: The Color is case sensitive: the keywords \a color, \a rgb, \a rgbf and \a random have
 * to be written in lower case characters.
 */
class PE_PUBLIC Color
{
private:
   //**Declarations for nested structures**********************************************************
   /*! \cond PE_INTERNAL */
   class String;
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend std::istream& operator>>( std::istream& is, Color& color );
   friend std::istream& operator>>( std::istream& is, Color::String& s );
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Color();
   explicit Color( real red, real green, real blue, real alpha=0.0 );
   explicit Color( const std::string& params );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void set( real red, real green, real blue, real alpha=0.0 );
   void set( const std::string& params );
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   bool parse( std::istream& is );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string color_;  //!< POV-Ray string representation of the color.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS COLOR::STRING
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for color values. */
class Color::String
{
private:
   //**Friend declarations*************************************************************************
   friend std::istream& operator>>( std::istream& is, Color::String& s );
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline String();
   explicit inline String( const std::string& s );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline const std::string& str()                         const;
   inline int                compare( const char* string ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string str_;  //!< String buffer.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Default constructor for the Color::String class.
 */
inline Color::String::String()
   : str_()  // String buffer
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Direct initialization constructor for the Color::String class.
 *
 * \param s The initialization string for the Color::String object.
 */
inline Color::String::String( const std::string& s )
   : str_( s )  // String buffer
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Conversion to a \a std::string.
 *
 * \return The converted \a std::string.
 */
inline const std::string& Color::String::str() const
{
   return str_;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Comparison between the String object and a C-style string.
 *
 * \param string The C-style string to be compared.
 * \return The result of the comparison.
 *
 * The function performes a lexicographic comparison between the String object and the C-style
 * string. The return value is either
 *
 * - less than 0 if the String object is smaller than the C-style string
 * - 0 if the String object is equal to the C-style string
 * - larger than 0 if the String object is larger than the C-style string
 */
inline int Color::String::compare( const char* string ) const
{
   return str_.compare( string );
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Color operators */
//@{
std::ostream& operator<<( std::ostream& os, const Color& color );
std::istream& operator>>( std::istream& is, Color& color );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\name Color::String operators */
//@{
std::istream& operator>>( std::istream& is, Color::String& s );
//@}
/*! \endcond */
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
