//=================================================================================================
/*!
 *  \file pe/core/joint/JointVector.h
 *  \brief Implementation of a vector for joint handles
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

#ifndef _PE_CORE_JOINT_JOINTVECTOR_H_
#define _PE_CORE_JOINT_JOINTVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for joint handles.
 * \ingroup core
 *
 * The JointVector class is a container class for joint handles of various type (e.g.
 * pe::FixedJoint, pe::HingeJoint, ...). It offers easy and fast access to the contained
 * joints:

   \code
   // Type definition for a Joint vector
   typedef pe::JointVector<Joint>  Joints;

   Joints joints;

   // Calculating the total number of joints contained in the joint vector.
   Joints::SizeType num = joints.size();

   // Loop over all joints contained in the joint vector.
   Joints::Iterator begin = joints.begin();
   Joints::Iterator end   = joints.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
template< typename J                    // Type of the joint
        , typename D = NoDelete         // Deletion policy
        , typename G = OptimalGrowth >  // Growth policy
class JointVector : public PtrVector<J,D,G>
{
private:
   //**Type definitions****************************************************************************
   typedef PtrVector<J,D,G>  Base;  //!< Type of the base class.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef J                     Joint;             //!< Type of the joint
   typedef J*                    PointerType;       //!< Pointer to a non-const object.
   typedef const J*              ConstPointerType;  //!< Pointer to a const object.
   typedef size_t                SizeType;          //!< Size type of the pointer vector.
   typedef PtrIterator<J>        Iterator;          //!< Iterator over non-const objects.
   typedef PtrIterator<const J>  ConstIterator;     //!< Iterator over const objects.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline JointVector( SizeType initCapacity = 100 );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool isActive() const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Standard constructor for JointVector.
 *
 * \param initCapacity The initial capacity of the contact vector.
 */
template< typename J    // Type of the joint
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline JointVector<J,D,G>::JointVector( SizeType initCapacity )
   : Base( initCapacity )  // Base class initialization
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the joint vector contains an active joint.
 *
 * \return \a true if at least one joint is active, \a false if all joints are inactive.
 *
 * The function returns whether the joint vector contains at least a single active joint or
 * if all the contained joints are inactive. A joint is considered to be active if at least
 * one of the attached rigid bodies is active.
 */
template< typename J    // Type of the joint
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline bool JointVector<J,D,G>::isActive() const
{
   for( ConstIterator j=this->begin(); j!=this->end(); ++j ) {
      if( j->isActive() ) return true;
   }
   return false;
}
//*************************************************************************************************

} // namespace pe

#endif
