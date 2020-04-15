//*************************************************************************************************
/*!
 *  \file src/core/contact/Contact.cpp
 *  \brief Source file for the Contact class.
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
//*************************************************************************************************


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactBase.h>
#include <pe/core/MPI.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

id_t ContactBase::nextContact_( 0 );




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the normal relative acceleration between the two contacting rigid bodies.
 *
 * \return The relative acceleration in normal direction.
 */
real ContactBase::getNormalRelAcc() const
{
   return trans(normal_) * ( b1_->accFromWF( gpos_ ) - b2_->accFromWF( gpos_ ) ) +
          real(2) * trans( getNDot() ) * ( b1_->velFromWF( gpos_ ) - b2_->velFromWF( gpos_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the normal relative force between the two contacting rigid bodies.
 *
 * \return The relative force in normal direction.
 */
real ContactBase::getNormalRelForce() const
{
   Vec3 force;

   if( !b1_->isFixed() ) force += b1_->getForce();
   if( !b2_->isFixed() ) force -= b2_->getForce();

   return ( trans(normal_) * force );
}
//*************************************************************************************************

} // namespace pe
