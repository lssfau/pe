//=================================================================================================
/*!
 *  \file pe/util/singleton/Dependency.h
 *  \brief Header file for the Dependency class
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

#ifndef _PE_UTIL_SINGLETON_DEPENDENCY_H_
#define _PE_UTIL_SINGLETON_DEPENDENCY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/shared_ptr.hpp>
#include <pe/util/constraints/DerivedFrom.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Lifetime dependency on a singleton object.
 * \ingroup singleton
 *
 * The Dependency template class represents a lifetime dependency on a singleton object based
 * on the \b pe Singleton functionality. By use of the Dependency template, any class can by
 * either public or non-public inheritance or composition define a single or multiple lifetime
 * dependencies on one or several singletons, which guarantees that the singleton instance(s)
 * will be destroyed after the dependent object. The following example demonstrates both the
 * inheritance as well as the composition approach:

   \code
   // Definition of the Viewer class, which is depending on the Logger singleton instance

   // #1: Approach by non-public inheritance
   class Viewer : private Dependency<Logger>
   {
      ...
   };

   // #2: Approach by composition
   class Viewer
   {
    private:
      Dependency<Logger> dependency_;
   };
   \endcode
 */
template< typename T >  // Type of the lifetime dependency
class Dependency
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline Dependency();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   boost::shared_ptr<T> dependency_;  //!< Handle to the lifetime dependency.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for Dependency.
 */
template< typename T >  // Type of the lifetime dependency
inline Dependency<T>::Dependency()
   : dependency_( T::instance() )  // Handle to the lifetime dependency
{
   pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, typename T::SingletonType );
}
//*************************************************************************************************

} // namespace pe

#endif
