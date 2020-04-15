//=================================================================================================
/*!
 *  \file pe/system/WarningDisable.h
 *  \brief Deactivation of compiler specific warnings
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

#ifndef _PE_SYSTEM_WARNINGDISABLE_H_
#define _PE_SYSTEM_WARNINGDISABLE_H_


//=================================================================================================
//
//  MICROSOFT VISUAL STUDIO WARNINGS
//
//=================================================================================================

#if defined(_MSC_VER) && (_MSC_VER >= 1400)

   // Disables a 'deprecated' warning for some standard library functions. This warning
   // is emitted when you use some perfectly conforming library functions in a perfectly
   // correct way, and also by some of Microsoft's own standard library code. For more
   // information about this particular warning, see
   // http://msdn.microsoft.com/en-us/library/ttcz0bys(VS.80).aspx
#  pragma warning(disable:4996)

   // Disables a warning for a this pointer that is passed to a base class in the constructor
   // initializer list.
#  pragma warning(disable:4355)

   // Disables the warning for ignored C++ exception specifications.
#  pragma warning(disable:4290)

#endif

#if defined(_MSC_VER) && (_MSC_VER <= 1600)

   // Disables the warning "unreferenced local function has been removed" in the case where
   // pure virtual destructors are defined and multiple inheritance is involved. In contrast
   // to what the warning claims, the code actually gets generated and called.
#  pragma warning(disable:4505)

#endif




//=================================================================================================
//
//  INTEL WARNINGS
//
//=================================================================================================

#if defined(__INTEL_COMPILER) || defined(__ICL)

   // Disables a 'deprecated' warning for some standard library functions.
#  pragma warning(disable:1786)

#endif

#endif
