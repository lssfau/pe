//=================================================================================================
/*!
 *  \file src/core/joint/JointBase.cpp
 *  \brief Source file for the JointBase class
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

#include <pe/core/joint/JointBase.h>
#include <pe/core/MPI.h>


namespace pe {




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the JointBase class.
 *
 * \param type The type of the joint.
 * \param rows Number of rows the joint occupies in Jacobian matrices.
 * \param body1 The first body to which the joint is attached.
 * \param body2 The second body to which the joint is attached.
 * \param scale Scaling parameter for visualization purposes \f$[0..1] \f$.
 * \param sid The unique system-specific ID.
 */
JointBase::JointBase( JointType type, size_t rows, BodyID body1, BodyID body2, real scale, id_t sid )
   : CCDJT ()                // Initialization of the coarse collision detection joint trait base class.
   , FCDJT ()                // Initialization of the fine collision detection joint trait base class
   , BJT   ()                // Initialization of the batch generation joint trait base class
   , RJT   ()                // Initialization of the collision response joint trait base class
   , type_ ( type )          // The type of the joint
   , rows_ ( rows )          // Number of rows the joint occupies in Jacobian matrices
   , sid_  ( sid )           // User-specific joint ID
   , index_( 0 )             // The current batch index of the joint
   , body1_( body1 )         // The first constrained rigid body
   , body2_( body2 )         // The second constrained rigid body
   , scale_( scale )         // Scaling parameter for visualization purposes
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the JointBase class.
 */
JointBase::~JointBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the scale of the joint in all active visualizations.
 *
 * \param scale The new scale of the joint \f$ [0..1] \f$.
 * \return void
 * \exception std::invalid_argument Invalid joint scale.
 *
 * TODO: explain the scale
 */
void JointBase::setScale( real scale )
{
   if( scale < real(0) || scale > real(1) )
      throw std::invalid_argument( "Invalid joint scale" );

   scale_ = scale;
}
//*************************************************************************************************

} // namespace pe
