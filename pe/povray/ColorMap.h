//=================================================================================================
/*!
 *  \file pe/povray/ColorMap.h
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

#ifndef _PE_POVRAY_COLORMAP_H_
#define _PE_POVRAY_COLORMAP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_colormap Color map
 * \ingroup povray
 */
/*!\brief A POV-Ray color map.
 * \ingroup povray_colormap
 *
 * A color map is a blend of colors over the range of a POV-Ray pigment. There are three different
 * ways to set up a color map:

   \code
   // Setup of a White/Red stripped color map
   pe::povray::ColorMap colormap( "[0.5 color White][0.5 color Red]" );  (1)
   colormap.set( "[0.5 color White][0.5 color Red]" );                   (2)
   is >> colormap;                                                       (3)
   \endcode

 * -# Option 1 is the direct setup via the ColorMap constructor. The parameter string has to
 *    contain the color map parameters according to the POV-Ray color map format (see below).
 * -# Option 2 is similar to the first setup, but can be used e.g. after a default setup of the
 *    color map via the default constructor. Again, the parameter string has to contain the color
 *    map parameters according to the POV-Ray color map format (see below).
 * -# Option 3 shows the setup of a color map via an input stream. The color map can be configured
 *    from any input stream containing the color map parameters according to the POV-Ray color map
 *    format (see below).\n
 *
 * To specify a color map with a color map parameter string, one of the following formats can be
 * used:

   \code
   [value1 color1] [value2 color2] [value3 color3]  (1)
   random                                           (3)
   Identifier                                       (2)
   \endcode

 * The first format directly specifies a certain color pattern. The value parameters are in the
 * range \f$ [0..1] \f$ and specify the location of the control points in the color map. For
 * each control point, a color is specified. Between the control points, linear interpolation
 * is used to calculate the color value (this also includes the transparency). In order to
 * specify a color map also note the following rules:
 *
 * - The number of control points has to be in the range \f$ [2..20] \f$.
 * - The value of a control point has to be greater or equal to the previous ones.
 * - If the first control point is greater than 0, all the colors between zero and that control
 *   point will be that color.
 * - Similarly, if the last control point is less than 1.0, all the colors between that control
 *   point and 1.0 will be that color.
 * - If two colors are specified to have the same control point, there will be a sudden change
 *   between colors at that point. The color at that point will be the color that is specified
 *   latest.
 * - If two control points have colors with different filter values, the filter values will be
 *   interpolated too, producing colors with intermediate transparency.
 * - The ColorMap is case sensitive: the \a color keyword has to be used in lower case characters.
 *
 * Instead of specifying a color pattern, the \a random keyword can used to create a random
 * color map consisting of 2 to 5 random colors. Additionally it is possible to specify a
 * predefined POV-Ray color map. This color map identifier is a single string and has to match
 * a declared POV-Ray color map exactly. Invalid identifiers will only be detected by POV-Ray
 * and not by the \b pe physics engine.
 *
 * Here are some examples of color maps:

   \code
   // Color blend from Red to green to blue and back to red
   [0.0 color Red]
   [0.33 color Green]
   [0.67 color Blue]
   [1.0 color Red]

   // A red/white stripped appearance
   [0.0 color Red]
   [0.5 color Red]
   [0.5 color White]
   [1.0 color White]

   // The same red/white stripped appearance, but shorter
   [0.5 color Red]
   [0.5 color White]
   \endcode
 */
class PE_PUBLIC ColorMap
{
private:
   //**Declarations for nested structures**********************************************************
   /*! \cond PE_INTERNAL */
   class String;
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend std::istream& operator>>( std::istream& is, ColorMap& colormap );
   friend std::istream& operator>>( std::istream& is, ColorMap::String& s );
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit ColorMap();
   explicit ColorMap( const std::string& params );
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

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   bool parse( std::istream& is );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string colormap_;  //!< POV-Ray string representation of the color map.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS COLORMAP::STRING
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for predefined POV-Ray color maps. */
class ColorMap::String
{
private:
   //**Friend declarations*************************************************************************
   friend std::istream& operator>>( std::istream& is, ColorMap::String& s );
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
/*!\brief Default constructor for the ColorMap::String class.
 */
inline ColorMap::String::String()
   : str_()  // String buffer
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Direct initialization constructor for the ColorMap::String class.
 *
 * \param s The initialization string for the ColorMap::String object.
 */
inline ColorMap::String::String( const std::string& s )
   : str_(s)  // String buffer
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Conversion to a \a std::string.
 *
 * \return The converted \a std::string.
 */
inline const std::string& ColorMap::String::str() const
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
inline int ColorMap::String::compare( const char* string ) const
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
/*!\name ColorMap operators */
//@{
std::ostream& operator<<( std::ostream& os, const ColorMap& colormap );
std::istream& operator>>( std::istream& is, ColorMap& colormap );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
