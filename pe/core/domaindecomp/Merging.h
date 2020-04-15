//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/Merging.h
 *  \brief Header file for the Merging class and the merge functions
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_DOMAINDECOMP_MERGING_H_
#define _PE_CORE_DOMAINDECOMP_MERGING_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <ostream>
#include <string>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/BaseOf.h>
#include <pe/util/NullType.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Merging/Union of two remote process gemetries.
 * \ingroup domaindecomp
 *
 * This class represents the merging of up to five process geometries. The process geometries
 * of type \a A, \a B, \a C, \a D, and \a E are merged together to form a new process geometry
 * consisting of the original geometries. A rigid body or a global coordinate contained in any
 * of the geometries the merging consists of is contained in the new geometry.\n
 * A merging of two to five process geometries can be created by one of the merge() functions:
 *
 * -# merge( const A& a, const B& b );
 * -# merge( const A& a, const B& b, const C& c );
 * -# merge( const A& a, const B& b, const C& c, const D& d );
 * -# merge( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The following example demonstrates the use of the merge() function with two given process
 * geometries:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Example for the merging of two half spaces" width=560pt
 */
template< typename A               // Type of the first process geometry
        , typename B               // Type of the second process geometry
        , typename C = NullType    // Type of the third process geometry
        , typename D = NullType    // Type of the fourth process geometry
        , typename E = NullType >  // Type of the fifth process geometry
class Merging : public ProcessGeometry
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Merging( const A& a, const B& b, const C& c, const D& d, const E& e );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Merging();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID     b ) const;
   virtual bool intersectsWith( ConstSphereID   s ) const;
   virtual bool intersectsWith( ConstBoxID      b ) const;
   virtual bool intersectsWith( ConstCapsuleID  c ) const;
   virtual bool intersectsWith( ConstCylinderID c ) const;
   virtual bool intersectsWith( ConstUnionID    u ) const;
   virtual bool containsPoint        ( const Vec3& gpos ) const;
   virtual bool containsPointStrictly( const Vec3& gpos ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions***************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os                  ) const;
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   A a_;  //!< The first merged process geometry.
   B b_;  //!< The second merged process geometry.
   C c_;  //!< The third merged process geometry.
   D d_;  //!< The fourth merged process geometry.
   E e_;  //!< The fifth merged process geometry.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, D );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, E );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Merging class.
 *
 * \param a The first merged process geometry.
 * \param b The second merged process geometry.
 * \param c The third merged process geometry.
 * \param d The fourth merged process geometry.
 * \param e The fifth merged process geometry.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
inline Merging<A,B,C,D,E>::Merging( const A& a, const B& b, const C& c, const D& d, const E& e )
   : a_( a )  // The first merged process geometry
   , b_( b )  // The second merged process geometry
   , c_( c )  // The third merged process geometry
   , d_( d )  // The fourth merged process geometry
   , e_( e )  // The fifth merged process geometry
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor of the Merging class.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
Merging<A,B,C,D,E>::~Merging()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the given rigid body intersects with the merged geometries.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the merged geometries, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the merging of
 * the process geometries. In case the body is partially contained in the merging the function
 * returns \a true, otherwise it returns \a false. Note that it is not possible to test infinite
 * rigid bodies (as for instance planes). The attempt to test an infinite rigid body results in
 * a \a std::invalid_argument exception.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstBodyID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) || c_.intersectsWith( b ) ||
            d_.intersectsWith( b ) || e_.intersectsWith( b ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sphere intersects with the merged geometries.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the merging
 * of the process geometries. In case the sphere is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstSphereID s ) const
{
   return ( a_.intersectsWith( s ) || b_.intersectsWith( s ) || c_.intersectsWith( s ) ||
            d_.intersectsWith( s ) || e_.intersectsWith( s ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given box intersects with the merged geometries.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given box is partially contained in the merging
 * of the process geometries. In case the box is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstBoxID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) || c_.intersectsWith( b ) ||
            d_.intersectsWith( b ) || e_.intersectsWith( b ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given capsule intersects with the merged geometries.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the merging
 * of the process geometries. In case the capsule is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstCapsuleID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) || c_.intersectsWith( c ) ||
            d_.intersectsWith( c ) || e_.intersectsWith( c ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given cylinder intersects with the merged geometries.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the merging
 * of the process geometries. In case the cylinder is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstCylinderID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) || c_.intersectsWith( c ) ||
            d_.intersectsWith( c ) || e_.intersectsWith( c ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given union intersects with the merged geometries.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given union is partially contained in the merging
 * of the process geometries. In case the union is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::intersectsWith( ConstUnionID u ) const
{
   return ( a_.intersectsWith( u ) || b_.intersectsWith( u ) || c_.intersectsWith( u ) ||
            d_.intersectsWith( u ) || e_.intersectsWith( u ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be part of it.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::containsPoint( const Vec3& gpos ) const
{
   return ( a_.containsPoint( gpos ) || b_.containsPoint( gpos ) ||
            c_.containsPoint( gpos ) || d_.containsPoint( gpos ) ||
            e_.containsPoint( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained strictly in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be \em not part of it. Note that if the merged geometries do not overlap
 * but only touch each other points on the joining surface  are consider to be \em not part of
 * the merged geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
bool Merging<A,B,C,D,E>::containsPointStrictly( const Vec3& gpos ) const
{
   return ( a_.containsPointStrictly( gpos ) || b_.containsPointStrictly( gpos ) ||
            c_.containsPointStrictly( gpos ) || d_.containsPointStrictly( gpos ) ||
            e_.containsPointStrictly( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts all half spaces in the process geometry.
 *
 * \param halfspaces Descriptions of the half spaces in the process geometry are appended to the list on return.
 * \return void
 *
 * The description of each half space is a pair of the half space normal and its signed distance from the origin. If the process geometry does not contain any half spaces no descriptions are appended.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
void Merging<A,B,C,D,E>::extractHalfSpaces( std::list< std::pair< Vec3, real > >& halfspaces ) const
{
   a_.extractHalfSpaces( halfspaces );
   b_.extractHalfSpaces( halfspaces );
   c_.extractHalfSpaces( halfspaces );
   d_.extractHalfSpaces( halfspaces );
   e_.extractHalfSpaces( halfspaces );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
void Merging<A,B,C,D,E>::print( std::ostream& os ) const
{
   os << "Merging:\n";
   a_.print( os, "   " );
   b_.print( os, "   " );
   c_.print( os, "   " );
   d_.print( os, "   " );
   e_.print( os, "   " );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the merging output.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
void Merging<A,B,C,D,E>::print( std::ostream& os, const char* tab ) const
{
   std::string longtab( tab );
   longtab.append( "   " );

   os << tab << "Merging:\n";
   a_.print( os, longtab.c_str() );
   b_.print( os, longtab.c_str() );
   c_.print( os, longtab.c_str() );
   d_.print( os, longtab.c_str() );
   e_.print( os, longtab.c_str() );
}
//*************************************************************************************************








//=================================================================================================
//
//  SPECIALIZATION FOR FOUR PROCESS GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Merging of four remote process geometries.
 * \ingroup domaindecomp
 *
 * This partial specialization of the Merging class template represents the merging of
 * exactly four process geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
class Merging<A,B,C,D,NullType> : public ProcessGeometry
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Merging( const A& a, const B& b, const C& c, const D& d );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Merging();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID     b ) const;
   virtual bool intersectsWith( ConstSphereID   s ) const;
   virtual bool intersectsWith( ConstBoxID      b ) const;
   virtual bool intersectsWith( ConstCapsuleID  c ) const;
   virtual bool intersectsWith( ConstCylinderID c ) const;
   virtual bool intersectsWith( ConstUnionID    u ) const;
   virtual bool containsPoint        ( const Vec3& gpos ) const;
   virtual bool containsPointStrictly( const Vec3& gpos ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions***************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os                  ) const;
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   A a_;  //!< The first merged process geometry.
   B b_;  //!< The second merged process geometry.
   C c_;  //!< The third merged process geometry.
   D d_;  //!< The fourth merged process geometry.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, D );
   /*! \endcond */
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Merging class.
 *
 * \param a The first merged process geometry.
 * \param b The second merged process geometry.
 * \param c The third merged process geometry.
 * \param d The fourth merged process geometry.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
inline Merging<A,B,C,D,NullType>::Merging( const A& a, const B& b, const C& c, const D& d )
   : a_( a )  // The first merged process geometry
   , b_( b )  // The second merged process geometry
   , c_( c )  // The third merged process geometry
   , d_( d )  // The fourth merged process geometry
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor of the Merging class.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
Merging<A,B,C,D,NullType>::~Merging()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given rigid body intersects with the merged geometries.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the merged geometries, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the merging of
 * the process geometries. In case the body is partially contained in the merging the function
 * returns \a true, otherwise it returns \a false. Note that it is not possible to test infinite
 * rigid bodies (as for instance planes). The attempt to test an infinite rigid body results in
 * a \a std::invalid_argument exception.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstBodyID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) ||
            c_.intersectsWith( b ) || d_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given sphere intersects with the merged geometries.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the merging
 * of the process geometries. In case the sphere is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstSphereID s ) const
{
   return ( a_.intersectsWith( s ) || b_.intersectsWith( s ) ||
            c_.intersectsWith( s ) || d_.intersectsWith( s ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given box intersects with the merged geometries.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given box is partially contained in the merging
 * of the process geometries. In case the box is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstBoxID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) ||
            c_.intersectsWith( b ) || d_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given capsule intersects with the merged geometries.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the merging
 * of the process geometries. In case the capsule is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstCapsuleID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) ||
            c_.intersectsWith( c ) || d_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given cylinder intersects with the merged geometries.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the merging
 * of the process geometries. In case the cylinder is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstCylinderID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) ||
            c_.intersectsWith( c ) || d_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given union intersects with the merged geometries.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given union is partially contained in the merging
 * of the process geometries. In case the union is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::intersectsWith( ConstUnionID u ) const
{
   return ( a_.intersectsWith( u ) || b_.intersectsWith( u ) ||
            c_.intersectsWith( u ) || d_.intersectsWith( u ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be part of it.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::containsPoint( const Vec3& gpos ) const
{
   return ( a_.containsPoint( gpos ) || b_.containsPoint( gpos ) ||
            c_.containsPoint( gpos ) || d_.containsPoint( gpos ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained strictly in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be \em not part of it. Note that if the merged geometries do not overlap
 * but only touch each other points on the joining surface  are consider to be \em not part of
 * the merged geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
bool Merging<A,B,C,D,NullType>::containsPointStrictly( const Vec3& gpos ) const
{
   return ( a_.containsPointStrictly( gpos ) || b_.containsPointStrictly( gpos ) ||
            c_.containsPointStrictly( gpos ) || d_.containsPointStrictly( gpos ) );
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts all half spaces in the process geometry.
 *
 * \param halfspaces Descriptions of the half spaces in the process geometry are appended to the list on return.
 * \return void
 *
 * The description of each half space is a pair of the half space normal and its signed distance from the origin. If the process geometry does not contain any half spaces no descriptions are appended.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
void Merging<A,B,C,D,NullType>::extractHalfSpaces( std::list< std::pair< Vec3, real > >& halfspaces ) const
{
   a_.extractHalfSpaces( halfspaces );
   b_.extractHalfSpaces( halfspaces );
   c_.extractHalfSpaces( halfspaces );
   d_.extractHalfSpaces( halfspaces );
}
//*************************************************************************************************



//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
void Merging<A,B,C,D,NullType>::print( std::ostream& os ) const
{
   os << "Merging:\n";
   a_.print( os, "   " );
   b_.print( os, "   " );
   c_.print( os, "   " );
   d_.print( os, "   " );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the merging output.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
void Merging<A,B,C,D,NullType>::print( std::ostream& os, const char* tab ) const
{
   std::string longtab( tab );
   longtab.append( "   " );

   os << tab << "Merging:\n";
   a_.print( os, longtab.c_str() );
   b_.print( os, longtab.c_str() );
   c_.print( os, longtab.c_str() );
   d_.print( os, longtab.c_str() );
}
/*! \endcond */
//*************************************************************************************************








//=================================================================================================
//
//  SPECIALIZATION FOR THREE PROCESS GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Merging of three remote process geometries.
 * \ingroup domaindecomp
 *
 * This partial specialization of the Merging class template represents the merging of
 * exactly three process geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
class Merging<A,B,C,NullType,NullType> : public ProcessGeometry
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Merging( const A& a, const B& b, const C& c );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Merging();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID     b ) const;
   virtual bool intersectsWith( ConstSphereID   s ) const;
   virtual bool intersectsWith( ConstBoxID      b ) const;
   virtual bool intersectsWith( ConstCapsuleID  c ) const;
   virtual bool intersectsWith( ConstCylinderID c ) const;
   virtual bool intersectsWith( ConstUnionID    u ) const;
   virtual bool containsPoint        ( const Vec3& gpos ) const;
   virtual bool containsPointStrictly( const Vec3& gpos ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions***************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os                  ) const;
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   A a_;  //!< The first merged process geometry.
   B b_;  //!< The second merged process geometry.
   C c_;  //!< The third merged process geometry.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );
   /*! \endcond */
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Merging class.
 *
 * \param a The first merged process geometry.
 * \param b The second merged process geometry.
 * \param c The third merged process geometry.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
inline Merging<A,B,C,NullType,NullType>::Merging( const A& a, const B& b, const C& c )
   : a_( a )  // The first merged process geometry
   , b_( b )  // The second merged process geometry
   , c_( c )  // The third merged process geometry
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor of the Merging class.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
Merging<A,B,C,NullType,NullType>::~Merging()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given rigid body intersects with the merged geometries.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the merged geometries, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the merging of
 * the process geometries. In case the body is partially contained in the merging the function
 * returns \a true, otherwise it returns \a false. Note that it is not possible to test infinite
 * rigid bodies (as for instance planes). The attempt to test an infinite rigid body results in
 * a \a std::invalid_argument exception.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstBodyID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) || c_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given sphere intersects with the merged geometries.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the merging
 * of the process geometries. In case the sphere is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstSphereID s ) const
{
   return ( a_.intersectsWith( s ) || b_.intersectsWith( s ) || c_.intersectsWith( s ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given box intersects with the merged geometries.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given box is partially contained in the merging
 * of the process geometries. In case the box is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstBoxID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) || c_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given capsule intersects with the merged geometries.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the merging
 * of the process geometries. In case the capsule is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstCapsuleID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) || c_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given cylinder intersects with the merged geometries.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the merging
 * of the process geometries. In case the cylinder is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstCylinderID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) || c_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given union intersects with the merged geometries.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given union is partially contained in the merging
 * of the process geometries. In case the union is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::intersectsWith( ConstUnionID u ) const
{
   return ( a_.intersectsWith( u ) || b_.intersectsWith( u ) || c_.intersectsWith( u ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be part of it.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::containsPoint( const Vec3& gpos ) const
{
   return ( a_.containsPoint( gpos ) || b_.containsPoint( gpos ) ||
            c_.containsPoint( gpos ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained strictly in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be \em not part of it. Note that if the merged geometries do not overlap
 * but only touch each other points on the joining surface  are consider to be \em not part of
 * the merged geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
bool Merging<A,B,C,NullType,NullType>::containsPointStrictly( const Vec3& gpos ) const
{
   return ( a_.containsPointStrictly( gpos ) || b_.containsPointStrictly( gpos ) ||
            c_.containsPointStrictly( gpos ) );
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts all half spaces in the process geometry.
 *
 * \param halfspaces Descriptions of the half spaces in the process geometry are appended to the list on return.
 * \return void
 *
 * The description of each half space is a pair of the half space normal and its signed distance from the origin. If the process geometry does not contain any half spaces no descriptions are appended.
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
void Merging<A,B,C,NullType,NullType>::extractHalfSpaces( std::list< std::pair< Vec3, real > >& halfspaces ) const
{
   a_.extractHalfSpaces( halfspaces );
   b_.extractHalfSpaces( halfspaces );
   c_.extractHalfSpaces( halfspaces );
}
//*************************************************************************************************



//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
void Merging<A,B,C,NullType,NullType>::print( std::ostream& os ) const
{
   os << "Merging:\n";
   a_.print( os, "   " );
   b_.print( os, "   " );
   c_.print( os, "   " );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the merging output.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
void Merging<A,B,C,NullType,NullType>::print( std::ostream& os, const char* tab ) const
{
   std::string longtab( tab );
   longtab.append( "   " );

   os << tab << "Merging:\n";
   a_.print( os, longtab.c_str() );
   b_.print( os, longtab.c_str() );
   c_.print( os, longtab.c_str() );
}
/*! \endcond */
//*************************************************************************************************








//=================================================================================================
//
//  SPECIALIZATION FOR TWO PROCESS GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Merging of two remote process geometries.
 * \ingroup domaindecomp
 *
 * This partial specialization of the Merging class template represents the merging of
 * exactly two process geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
class Merging<A,B,NullType,NullType,NullType> : public ProcessGeometry
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline Merging( const A& a, const B& b );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Merging();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool intersectsWith( ConstBodyID     b ) const;
   virtual bool intersectsWith( ConstSphereID   s ) const;
   virtual bool intersectsWith( ConstBoxID      b ) const;
   virtual bool intersectsWith( ConstCapsuleID  c ) const;
   virtual bool intersectsWith( ConstCylinderID c ) const;
   virtual bool intersectsWith( ConstUnionID    u ) const;
   virtual bool containsPoint        ( const Vec3& gpos ) const;
   virtual bool containsPointStrictly( const Vec3& gpos ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions***************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual void extractHalfSpaces( std::list< std::pair<Vec3, real> >& halfspaces ) const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os                  ) const;
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   A a_;  //!< The first merged process geometry.
   B b_;  //!< The second merged process geometry.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   /*! \endcond */
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Merging class.
 *
 * \param a The first merged process geometry.
 * \param b The second merged process geometry.
 * \param c The third merged process geometry.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
inline Merging<A,B,NullType,NullType,NullType>::Merging( const A& a, const B& b )
   : a_( a )  // The first merged process geometry
   , b_( b )  // The second merged process geometry
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor of the Merging class.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
Merging<A,B,NullType,NullType,NullType>::~Merging()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given rigid body intersects with the merged geometries.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the merged geometries, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the merging of
 * the process geometries. In case the body is partially contained in the merging the function
 * returns \a true, otherwise it returns \a false. Note that it is not possible to test infinite
 * rigid bodies (as for instance planes). The attempt to test an infinite rigid body results in
 * a \a std::invalid_argument exception.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstBodyID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given sphere intersects with the merged geometries.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the merging
 * of the process geometries. In case the sphere is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstSphereID s ) const
{
   return ( a_.intersectsWith( s ) || b_.intersectsWith( s ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given box intersects with the merged geometries.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given box is partially contained in the merging
 * of the process geometries. In case the box is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstBoxID b ) const
{
   return ( a_.intersectsWith( b ) || b_.intersectsWith( b ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given capsule intersects with the merged geometries.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the merging
 * of the process geometries. In case the capsule is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstCapsuleID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given cylinder intersects with the merged geometries.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the merging
 * of the process geometries. In case the cylinder is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstCylinderID c ) const
{
   return ( a_.intersectsWith( c ) || b_.intersectsWith( c ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether the given union intersects with the merged geometries.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the merged geometries, \a false if not.
 *
 * This function tests whether the given union is partially contained in the merging
 * of the process geometries. In case the union is partially contained in the merging
 * the function returns \a true, otherwise it returns \a false.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::intersectsWith( ConstUnionID u ) const
{
   return ( a_.intersectsWith( u ) || b_.intersectsWith( u ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be part of it.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::containsPoint( const Vec3& gpos ) const
{
   return ( a_.containsPoint( gpos ) || b_.containsPoint( gpos ) );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Tests if a global coordinate is contained strictly in the merging.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the merging, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the merging of
 * the process geometries. If the point is located on the surface of the merged geometries then
 * it is considered to be \em not part of it. Note that if the merged geometries do not overlap
 * but only touch each other points on the joining surface  are consider to be \em not part of
 * the merged geometries.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
bool Merging<A,B,NullType,NullType,NullType>::containsPointStrictly( const Vec3& gpos ) const
{
   return ( a_.containsPointStrictly( gpos ) || b_.containsPointStrictly( gpos ) );
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts all half spaces in the process geometry.
 *
 * \param halfspaces Descriptions of the half spaces in the process geometry are appended to the list on return.
 * \return void
 *
 * The description of each half space is a pair of the half space normal and its signed distance from the origin. If the process geometry does not contain any half spaces no descriptions are appended.
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
void Merging<A,B,NullType,NullType,NullType>::extractHalfSpaces( std::list< std::pair< Vec3, real > >& halfspaces ) const
{
   a_.extractHalfSpaces( halfspaces );
   b_.extractHalfSpaces( halfspaces );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
void Merging<A,B,NullType,NullType,NullType>::print( std::ostream& os ) const
{
   os << "Merging:\n";
   a_.print( os, "   " );
   b_.print( os, "   " );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Output of the merging properties.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the merging output.
 * \return void
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
void Merging<A,B,NullType,NullType,NullType>::print( std::ostream& os, const char* tab ) const
{
   std::string longtab( tab );
   longtab.append( "   " );

   os << tab << "Merging:\n";
   a_.print( os, longtab.c_str() );
   b_.print( os, longtab.c_str() );
}
/*! \endcond */
//*************************************************************************************************








//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Merging functions */
//@{
template< typename A, typename B >
inline Merging<A,B>
   merge( const A& a, const B& b );

template< typename A, typename B, typename C >
inline Merging<A,B,C>
   merge( const A& a, const B& b, const C& c );

template< typename A, typename B, typename C, typename D >
inline Merging<A,B,C,D>
   merge( const A& a, const B& b, const C& c, const D& d );

template< typename A, typename B, typename C, typename D, typename E >
inline Merging<A,B,C,D,E>
   merge( const A& a, const B& b, const C& c, const D& d, const E& e );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging two process geometries.
 * \ingroup domaindecomp
 *
 * \param a The first process geometry.
 * \param b The second process geometry.
 * \return The merging of the two given process geometries.
 *
 * This function creates a merging of the two given process geometries. The following example
 * demonstrates the use of the merge() function:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Example for the merging of two half spaces" width=560pt
 */
template< typename A    // Type of the first process geometry
        , typename B >  // Type of the second process geometry
inline Merging<A,B> merge( const A& a, const B& b )
{
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );

   return Merging<A,B>( a, b );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging three process geometries.
 * \ingroup domaindecomp
 *
 * \param a The first process geometry.
 * \param b The second process geometry.
 * \param c The third process geometry.
 * \return The merging of the three given process geometries.
 *
 * This function creates a merging of the three given process geometries. The following example
 * demonstrates the use of the merge() function with two process geometries:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Example for the merging of two half spaces" width=560pt
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C >  // Type of the third process geometry
inline Merging<A,B,C> merge( const A& a, const B& b, const C& c )
{
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );

   return Merging<A,B,C>( a, b, c );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging four process geometries.
 * \ingroup domaindecomp
 *
 * \param a The first process geometry.
 * \param b The second process geometry.
 * \param c The third process geometry.
 * \param d The fourth process geometry.
 * \return The merging of the four given process geometries.
 *
 * This function creates a merging of the four given process geometries. The following example
 * demonstrates the use of the merge() function with two process geometries:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Example for the merging of two half spaces" width=560pt
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D >  // Type of the fourth process geometry
inline Merging<A,B,C,D> merge( const A& a, const B& b, const C& c, const D& d )
{
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, D );

   return Merging<A,B,C,D>( a, b, c, d );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Merging five process geometries.
 * \ingroup domaindecomp
 *
 * \param a The first process geometry.
 * \param b The second process geometry.
 * \param c The third process geometry.
 * \param d The fourth process geometry.
 * \param e The fifth process geometry.
 * \return The merging of the five given process geometries.
 *
 * This function creates a merging of the five given process geometries. The following example
 * demonstrates the use of the merge() function with two process geometries:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Example for the merging of two half spaces" width=560pt
 */
template< typename A    // Type of the first process geometry
        , typename B    // Type of the second process geometry
        , typename C    // Type of the third process geometry
        , typename D    // Type of the fourth process geometry
        , typename E >  // Type of the fifth process geometry
inline Merging<A,B,C,D,E> merge( const A& a, const B& b, const C& c, const D& d, const E& e )
{
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, A );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, B );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, C );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, D );
   pe_CONSTRAINT_MUST_BE_BASE_OF( ProcessGeometry, E );

   return Merging<A,B,C,D,E>( a, b, c, d, e );
}
//*************************************************************************************************

} // namespace pe

#endif
