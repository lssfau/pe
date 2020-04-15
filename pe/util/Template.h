//=================================================================================================
/*!
 *  \file pe/util/Template.h
 *  \brief Header file for nested template disabiguation
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

#ifndef _PE_UTIL_TEMPLATE_H_
#define _PE_UTIL_TEMPLATE_H_


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compiler specific patch for nested template disambiguation.
 * \ingroup util
 *
 * The pe_TEMPLATE is a patch for the Microsoft Visual C++ compiler that does not correctly
 * parse definitions of nested templates of the following form:

   \code
   template< typename T >
   class Alloc {
    public:
      ...
      template< typename Other >
      class rebind {
       public:
         typedef Alloc<Other> other;
      };
      ...
   };

   typedef Alloc<int>  AI;
   typedef AI::template rebind<double>::other  Other;  // Compilation error with Visual C++
   \endcode

 * In order to circumvent this compilation error, the pe_TEMPLATE macro should be used instead
 * the \a template keyword:

   \code
   ...
   typedef AI::pe_TEMPLATE rebind<double>::other  Other;  // No compilation errors
   \endcode
 */
#if defined(_MSC_VER)
#  define pe_TEMPLATE
#else
#  define pe_TEMPLATE template
#endif
/*! \endcond */
//*************************************************************************************************

#endif
