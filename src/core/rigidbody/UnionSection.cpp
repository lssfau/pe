//=================================================================================================
/*!
 *  \file src/core/rigidbody/UnionSection.cpp
 *  \brief Souirce file for the Union setup environments
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

#include <stdexcept>
#include <pe/core/BodyManager.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

size_t CreateUnion::counter_     ( 0 );
size_t InstantiateUnion::counter_( 0 );




//=================================================================================================
//
//  CLASS CREATEUNION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the CreateUnion class.
 */
CreateUnion::CreateUnion( id_t id )
   : defaultManager_( theDefaultManager() )  // The previously instated default body manager
   , union_         ( createUnion( id ) )    // The new union
{
   // Instating the union as the default body manager
   union_->setDefaultManager();

   // Incrementing the union section counter
   ++counter_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destructor of the CreateUnion class.
 *
 * \exception std::invalid_argument Invalid global union position.
 */
#if (_MSC_VER >= 1900) // Visual Studio 2015 or newer
   CreateUnion::~CreateUnion() noexcept(false) // Disable implicit noexcept
#else
 CreateUnion::~CreateUnion()
#endif
{
   // Checking the union manager and the union section counter
   pe_INTERNAL_ASSERT( union_->getManager() == defaultManager_, "Invalid body manager detected" );
   pe_INTERNAL_ASSERT( counter_ > 0, "Invalid union section counter" );

   // Restoring the old default body manager
   defaultManager_->setDefaultManager();

   // Decrementing the union section counter
   --counter_;

   // In case of a currently active exception, destroy the newly created union
   if( std::uncaught_exception() ) {
      destroy( union_ );
      return;
   }

   // Checking the global position of the union
   if( counter_ == 0 && !GlobalSection::isActive() && !theCollisionSystem()->getDomain().ownsPoint( union_->getPosition() ) ) {
      destroy( union_ );
      pe_LOG_ERROR_SECTION(log){
         log << "Invalid global union position";
      }
      exit(EXIT_FAILURE);
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS INSTANTIATEUNION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the InstantiateUnion class.
 */
InstantiateUnion::InstantiateUnion( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, real mass,
                                    const Mat3& I, const Quat& q, const Vec6& aabb, bool visible, bool fixed, bool reg )
   : defaultManager_( theDefaultManager() )  // The previously instated default body manager
   , union_         ( 0 )                    // The new union
{
   // Incrementing the union section counter
   ++counter_;

   // Instantiating a new remote union
   union_ = instantiateUnion( sid, uid, gpos, rpos, mass, I, q, aabb, visible, fixed, reg );

   // Instating the union as the default body manager
   union_->setDefaultManager();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destructor of the InstantiateUnion class.
 */
InstantiateUnion::~InstantiateUnion()
{
   // Checking the union manager and the union section counter
   pe_INTERNAL_ASSERT( union_->getManager() == defaultManager_ || union_->getManager() == 0, "Invalid body manager detected" );
   pe_INTERNAL_ASSERT( counter_ > 0, "Invalid pe_INSTANTIATE_UNION section counter" );

   // Restoring the old default body manager
   defaultManager_->setDefaultManager();

   // Decrementing the union section counter
   --counter_;

   // In case of a currently active exception, destroy the newly created union
   if( std::uncaught_exception() ) {
      destroy( union_ );
      return;
   }
}
//*************************************************************************************************

} // namespace pe
