//=================================================================================================
/*!
 *  \file pe/core/response/FrictionlessSolver.h
 *  \brief Constructs system matrices and right-hand-sides for frictionless contact problems
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

#ifndef _PE_CORE_RESPONSE_FRICTIONLESSCONSTRUCTOR_H_
#define _PE_CORE_RESPONSE_FRICTIONLESSCONSTRUCTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <pe/math/Matrix3x3.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/util/Assert.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 * \ingroup collision_response
 *
 * TODO
 */
template< typename C >  // Type of the configuration
class FrictionlessConstructor : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef typename Config::ContactID       ContactID;       //!< Handle to a contact.
   typedef typename Config::ConstContactID  ConstContactID;  //!< Handle to a contact.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   template< typename CP >
   explicit inline FrictionlessConstructor( CP& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**System construction functions***************************************************************
   /*!\name System construction functions */
   //@{
   inline void allocate( const std::vector<size_t>& capacities );
   inline void insertMatrixOffdiag( size_t i, size_t j, ConstContactID c_i, ConstContactID c_j, const Mat3& JtMinvJ_ij );
   inline void insertMatrixDiag( size_t i, ConstContactID c_i, const Mat3& JtMinvJ_ii );
   inline void insertVector( size_t i, ConstContactID c_i, const Vec3& Jtb );

   template< typename Contacts >
   inline void postprocess( const Contacts& contacts );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   SMatN& A_;  //!< The system matrix \f$ A \f$.
   VecN&  b_;  //!< The right-hand side vector \f$ b \f$.
   VecN&  x_;  //!< The vector of unknowns \f$ x \f$.
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
/*!\brief Constructor for the FrictionlessConstructor class.
 *
 * \param cp TODO
 */
template< typename C >   // Type of the configuration
template< typename CP >  // Type of the complementarity system
inline FrictionlessConstructor<C>::FrictionlessConstructor( CP& cp )
   : A_( cp.A_ )  // The system matrix A
   , b_( cp.b_ )  // The right-hand side vector b
   , x_( cp.x_ )  // The vector of unknowns x
{}
//*************************************************************************************************




//=================================================================================================
//
//  SYSTEM CONSTRUCTION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * \param nonzeros TODO
 *
 * TODO
 */
template< typename C >  // Type of the configuration
inline void FrictionlessConstructor<C>::allocate( const std::vector<size_t>& nonzeros )
{
   // Creating a new square system matrix
   SMatN( nonzeros.size(), nonzeros.size(), nonzeros ).swap( A_ );

   // Resizing the right-hand side vector and the vector of unknwons
   b_.resize( nonzeros.size(), false );
   x_.resize( nonzeros.size(), false );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i TODO
 * \param j TODO
 * \param c_i TODO
 * \param c_j TODO
 * \param JtMinvJ_ij TODO
 *
 * TODO
 */
template< typename C >  // Type of the configuration
inline void FrictionlessConstructor<C>::insertMatrixOffdiag( size_t i, size_t j, ConstContactID c_i, ConstContactID c_j, const Mat3& JtMinvJ_ij )
{
   const Vec3& n_i( c_i->getNormal() );
   const Vec3& n_j( c_j->getNormal() );
   const real NtJtMinvJN_ij( trans(n_i)  * ( JtMinvJ_ij * n_j ) );

   if( NtJtMinvJN_ij != real(0) ) {
      A_.append( i, j, NtJtMinvJN_ij );
      A_.append( j, i, NtJtMinvJN_ij );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i TODO
 * \param c_i TODO
 * \param JtMinvJ_ij TODO
 *
 * TODO
 */
template< typename C >  // Type of the configuration
inline void FrictionlessConstructor<C>::insertMatrixDiag( size_t i, ConstContactID c_i, const Mat3& JtMinvJ_ij )
{
   const Vec3& n_i ( c_i->getNormal() );
   const real NtJtMinvJN_ii( trans(n_i)  * ( JtMinvJ_ij * n_i ) );

   pe_INTERNAL_ASSERT( NtJtMinvJN_ii != real(0), "Invalid zero diagonal element detected" );
   A_.append( i, i, NtJtMinvJN_ii );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i TODO
 * \param c_i TODO
 * \param Jtb TODO
 *
 * TODO
 */
template< typename C >  // Type of the configuration
inline void FrictionlessConstructor<C>::insertVector( size_t i, ConstContactID c_i, const Vec3& Jtb )
{
   b_[i] = trans(Jtb) * c_i->getNormal() + c_i->getRestitution() * c_i->getNormalRelVel();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param contacts TODO
 *
 * TODO
 */
template< typename C >    // Type of the configuration
template< typename Contacts >  // Contact container type
inline void FrictionlessConstructor<C>::postprocess( const Contacts& /*contacts*/ )
{
   // Initialization of the vector of unknowns
   x_ = real(0);
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
