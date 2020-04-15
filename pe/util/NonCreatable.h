//=================================================================================================
/*!
 *  \file pe/util/NonCreatable.h
 *  \brief Base class for non-creatable (static) classes
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

#ifndef _PE_UTIL_NONCREATABLE_H_
#define _PE_UTIL_NONCREATABLE_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for non-creatable (static) classes.
 * \ingroup util
 *
 * The NonCreatable class is intended to work as a base class for non-creatable classes, i.e.
 * classes that cannot be instantiated and exclusively offer static functions/data. Both the
 * standard as well as the copy constructor and the copy assignment operator are declared
 * private and left undefinded in order to prohibit the instantiation of objects of derived
 * classes.\n
 *
 * \b Note: It is not necessary to publicly derive from this class. It is sufficient to derive
 * privately to prevent the instantiation of the derived class.

   \code
   class A : private NonCreatable
   { ... };
   \endcode
 */
class NonCreatable
{
private:
   //**Constructors and copy assignment operator***************************************************
   /*!\name Constructors and copy assignment operator */
   //@{
   NonCreatable();                                  //!< Constructor (private & undefined)
   NonCreatable( const NonCreatable& );             //!< Copy constructor (private & undefined)
   NonCreatable& operator=( const NonCreatable& );  //!< Copy assignment operator (private & undefined)
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
