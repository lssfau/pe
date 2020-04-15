//=================================================================================================
/*!
 *  \file pe/core/joint/JointTrait.h
 *  \brief Header file for the JointTrait class
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

#ifndef _PE_CORE_JOINT_JOINTTRAIT_H_
#define _PE_CORE_JOINT_JOINTTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>
#include <pe/core/joint/JointBase.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Joint customization class for the collision response.
 * \ingroup joints
 *
 * The JointTrait class template is a customization class for joints in general. Its main
 * purpose is the customization of the Joint class for the selected collision response
 * algorithm (see pe::pe_CONSTRAINT_SOLVER).\n
 * Depending on the used algorithm, a joint may require additional data or functionality
 * to efficiently support the collision response calculations. In order to add this specific
 * functionality or data and to adapt joints to a particular algorithm, the base template
 * needs to be specialized.
 */
template< typename C >  // Type of the configuration
class JointTrait : public JointBase
{
private:
   //**Type definitions****************************************************************************
   typedef JointBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline JointTrait( JointType type, size_t rows, BodyID body1, BodyID body2, real scale, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~JointTrait() = 0;
   //@}
   //**********************************************************************************************

};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the JointTrait constructor.
 *
 * \param type The type of the joint.
 * \param rows Number of rows the joint occupies in Jacobian matrices.
 * \param body1 The first body to which the joint is attached.
 * \param body2 The second body to which the joint is attached.
 * \param scale Scaling parameter for visualization purposes \f$[0..1] \f$.
 * \param sid Unique system ID.
 */
template< typename C >  // Type of the configuration
inline JointTrait<C>::JointTrait( JointType type, size_t rows, BodyID body1, BodyID body2, real scale, id_t sid )
   : Parent( type, rows, body1, body2, scale, sid )  // Initialization of the parent object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the JointTrait destructor.
 */
template< typename C >  // Type of the configuration
JointTrait<C>::~JointTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
