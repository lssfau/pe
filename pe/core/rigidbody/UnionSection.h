//=================================================================================================
/*!
 *  \file pe/core/rigidbody/UnionSection.h
 *  \brief Header file for the Union setup environment
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

#ifndef _PE_CORE_RIGIDBODY_UNIONSECTION_H_
#define _PE_CORE_RIGIDBODY_UNIONSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <new>
#include <pe/core/Types.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/math/Vector3.h>
#include <pe/math/Vector6.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS CREATEUNION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Auxiliary class for the pe_CREATE_UNION environment.
 * \ingroup union
 *
 * The CreateUnion class is an auxiliary helper class for the \a pe_CREATE_UNION macro.
 * It provides the functionality to instate the newly created union as the default body
 * manager, reinstate the previous default manager after the setup of the union, and
 * additionally works as a handle to the new union.
 */
class CreateUnion
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   CreateUnion( id_t id );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
#if (_MSC_VER >= 1900) // Visual Studio 2015 or newer
   ~CreateUnion() noexcept(false); // Disable implicit noexcept
#else
   ~CreateUnion();
#endif
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool isActive();
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline UnionID operator->() const;
   inline operator UnionID() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ManagerID defaultManager_;  //!< The previously instated default body manager.
                               /*!< After the setup of the union, this body manager will be
                                    reinstated as the default body manager. */
   UnionID union_;             //!< The new union.
   static size_t counter_;     //!< Union section counter.
                               /*!< The counter corresponds to the number of currently nested
                                    union sections. In case the counter is 0, the currently
                                    executed code is not inside an union section. Otherwise
                                    it is executed inside a (nested) union section. */
   //@}
   //**********************************************************************************************

   //**Forbidden operations************************************************************************
   /*!\name Forbidden operations */
   //@{
   CreateUnion& operator=( const CreateUnion& );

   void* operator new  ( std::size_t );
   void* operator new[]( std::size_t );
   void* operator new  ( std::size_t, const std::nothrow_t& ) PE_NOTHROW;
   void* operator new[]( std::size_t, const std::nothrow_t& ) PE_NOTHROW;

   void operator delete  ( void* ) PE_NOTHROW;
   void operator delete[]( void* ) PE_NOTHROW;
   void operator delete  ( void*, const std::nothrow_t& ) PE_NOTHROW;
   void operator delete[]( void*, const std::nothrow_t& ) PE_NOTHROW;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether an union section is active or not.
 *
 * \return \a true if an union section is active, \a false if not.
 */
inline bool CreateUnion::isActive()
{
   return counter_ > size_t(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a pointer to the newly created union.
 *
 * \return Pointer to the new union.
 */
inline UnionID CreateUnion::operator->() const
{
   return union_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a pointer to the newly created union.
 *
 * \return Pointer to the new union.
 */
inline CreateUnion::operator UnionID() const
{
   return union_;
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS INSTANTIATEUNION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Auxiliary class for the pe_INSTANTIATE_UNION environment.
 * \ingroup union
 *
 * The InstantiateUnion class is an auxiliary helper class for the \a pe_INSTANTIATE_UNION
 * macro. It provides the functionality to instate the newly created union as the default
 * body manager, reinstate the previous default manager after the setup of the union, and
 * additionally works as a handle to the new union.
 */
class InstantiateUnion
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   InstantiateUnion( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, real mass,
                     const Mat3& I, const Quat& q, const Vec6& aabb, bool visible, bool fixed,
                     bool reg=true );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~InstantiateUnion();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool isActive();
   //@}
   //**********************************************************************************************

   //**Access operators****************************************************************************
   /*!\name Access operators */
   //@{
   inline UnionID operator->() const;
   inline operator UnionID() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ManagerID defaultManager_;  //!< The previously instated default body manager.
                               /*!< After the setup of the union, this body manager will be
                                    reinstated as the default body manager. */
   UnionID union_;             //!< The new union.
   static size_t counter_;     //!< Union section counter.
                               /*!< The counter corresponds to the number of currently nested
                                    union sections. In case the counter is 0, the currently
                                    executed code is not inside an union section. Otherwise
                                    it is executed inside a (nested) union section. */
   //@}
   //**********************************************************************************************

   //**Forbidden operations************************************************************************
   /*!\name Forbidden operations */
   //@{
   InstantiateUnion& operator=( const InstantiateUnion& );

   void* operator new  ( std::size_t );
   void* operator new[]( std::size_t );
   void* operator new  ( std::size_t, const std::nothrow_t& ) PE_NOTHROW;
   void* operator new[]( std::size_t, const std::nothrow_t& ) PE_NOTHROW;

   void operator delete  ( void* ) PE_NOTHROW;
   void operator delete[]( void* ) PE_NOTHROW;
   void operator delete  ( void*, const std::nothrow_t& ) PE_NOTHROW;
   void operator delete[]( void*, const std::nothrow_t& ) PE_NOTHROW;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether a pe_INSTANTIATE_UNION section is active or not.
 *
 * \return \a true if a pe_INSTANTIATE_UNION section is active, \a false if not.
 */
inline bool InstantiateUnion::isActive()
{
   return counter_ > size_t(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a pointer to the newly created union.
 *
 * \return Pointer to the new union.
 */
inline UnionID InstantiateUnion::operator->() const
{
   return union_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a pointer to the newly created union.
 *
 * \return Pointer to the new union.
 */
inline InstantiateUnion::operator UnionID() const
{
   return union_;
}
//*************************************************************************************************




//=================================================================================================
//
//  CREATE UNION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup and configuration of a union compound geometry.
 * \ingroup union
 *
 * This macro provides a convenient environment to setup and configure a union compound geometry.
 * The following example demonstrates the use of the pe_CREATE_UNION environment:

   \code
   int main( int argc, char** argv )
   {
      // ...

      // Setup of a union compound geometry
      // This command starts a new section for the setup and configuration of a new union
      // object. All rigid bodies created within the following section will be automatically
      // added to the new union. The new union itself is available via the name specified as
      // the first argument of the macro. The second argument specifies the user-specific ID.
      pe_CREATE_UNION( union1, 1 )
      {
         // Creating an iron sphere. The sphere is directly added to the union 1
         SphereID sphere1 = createSphere( 1, 2.0, -3.0, 0.5, 1.0, iron );

         // Creating an oak box at the origin, which is also directly added to union 1.
         // Afterward the box is translated next to sphere 1.
         BoxID box2 = createBox( 2, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, oak );
         box2->translate( 4.0, -3.0, 0.5 );

         // Creating a link between sphere 1 and the box 2
         LinkID link1 = createLink( union1, 1, sphere1, box2 );

         // Rotating the entire union around the y-axis
         union1->rotate( 0.0, PI/3.45, 0.0 );
      }

      // ...
   }
   \endcode

 * In case any of the function calls inside the pe_CREATE_UNION section results in an exception
 * being thrown, which is not caught inside the section, the entire union is destroyed!
 */
#define pe_CREATE_UNION( HANDLE, ID ) \
   if( CreateUnion HANDLE = (ID) )
//*************************************************************************************************




//=================================================================================================
//
//  INSTANTIATE UNION MACRO
//
//=================================================================================================

//*************************************************************************************************
// TODO
#define pe_INSTANTIATE_UNION( HANDLE, SID, UID, GPOS, RPOS, MASS, I, Q, AABB, VISIBLE, FIXED, REG ) \
   if( InstantiateUnion HANDLE = InstantiateUnion( SID, UID, GPOS, RPOS, MASS, I, Q, AABB, VISIBLE, FIXED, REG ) )
//*************************************************************************************************

} // namespace pe

#endif
