//=================================================================================================
/*!
 *  \file pe/core/response/FrictionConstructor.h
 *  \brief Constructs system matrices and right-hand-sides for frictional contact problems
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

#ifndef _PE_CORE_RESPONSE_FRICTIONCONSTRUCTOR_H_
#define _PE_CORE_RESPONSE_FRICTIONCONSTRUCTOR_H_


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
class FrictionConstructor : private NonCopyable
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
   explicit inline FrictionConstructor( CP& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**System construction functions***************************************************************
   /*!\name System construction functions */
   //@{
   inline void allocate( const std::vector<size_t>& capacities );
          void insertMatrixOffdiag( size_t i, size_t j, ConstContactID c_i, ConstContactID c_j, const Mat3& JtMinvJ_ij );
          void insertMatrixDiag( size_t i, ConstContactID c_i, const Mat3& JtMinvJ_ii );
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
inline FrictionConstructor<C>::FrictionConstructor( CP& cp )
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
inline void FrictionConstructor<C>::allocate( const std::vector<size_t>& nonzeros )
{
   // Tripling the number of rows and columns
   const size_t n( nonzeros.size() * 3 );
   std::vector<size_t> tmp( n );

   for( size_t i=0; i<nonzeros.size(); ++i ) {
      tmp[3*i] = tmp[3*i+1] = tmp[3*i+2] = 3 * nonzeros[i];
   }

   // Creating a new square system matrix
   SMatN( n, n, tmp ).swap( A_ );

   // Resizing the right-hand side vector and the vector of unknwons
   b_.resize( n, false );
   x_.resize( n, false );
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
void FrictionConstructor<C>::insertMatrixOffdiag( size_t i, size_t j, ConstContactID c_i, ConstContactID c_j, const Mat3& JtMinvJ_ij )
{
   const Vec3& n_i ( c_i->getNormal()   );
   const Vec3& n_j ( c_j->getNormal()   );
   const Vec3& tx_i( c_i->getTangentX() );
   const Vec3& tx_j( c_j->getTangentX() );
   const Vec3& ty_i( c_i->getTangentY() );
   const Vec3& ty_j( c_j->getTangentY() );

   const real NtJtMinvJN_ij  ( trans(n_i)  * ( JtMinvJ_ij * n_j  ) );
   const real NtJtMinvJTx_ij ( trans(n_i)  * ( JtMinvJ_ij * tx_j ) );
   const real NtJtMinvJTy_ij ( trans(n_i)  * ( JtMinvJ_ij * ty_j ) );
   const real TxtJtMinvJN_ij ( trans(tx_i) * ( JtMinvJ_ij * n_j  ) );
   const real TxtJtMinvJTx_ij( trans(tx_i) * ( JtMinvJ_ij * tx_j ) );
   const real TxtJtMinvJTy_ij( trans(tx_i) * ( JtMinvJ_ij * ty_j ) );
   const real TytJtMinvJN_ij ( trans(ty_i) * ( JtMinvJ_ij * n_j  ) );
   const real TytJtMinvJTx_ij( trans(ty_i) * ( JtMinvJ_ij * tx_j ) );
   const real TytJtMinvJTy_ij( trans(ty_i) * ( JtMinvJ_ij * ty_j ) );

   if( NtJtMinvJN_ij   != real(0) ) {
      A_.append( 3*i,   3*j,   NtJtMinvJN_ij   );
      A_.append( 3*j,   3*i,   NtJtMinvJN_ij   );
   }
   if( NtJtMinvJTx_ij  != real(0) ) {
      A_.append( 3*i,   3*j+1, NtJtMinvJTx_ij  );
      A_.append( 3*j+1, 3*i,   NtJtMinvJTx_ij  );
   }
   if( NtJtMinvJTy_ij  != real(0) ) {
      A_.append( 3*i,   3*j+2, NtJtMinvJTy_ij  );
      A_.append( 3*j+2, 3*i,   NtJtMinvJTy_ij  );
   }
   if( TxtJtMinvJN_ij  != real(0) ) {
      A_.append( 3*i+1, 3*j,   TxtJtMinvJN_ij  );
      A_.append( 3*j,   3*i+1, TxtJtMinvJN_ij  );
   }
   if( TxtJtMinvJTx_ij != real(0) ) {
      A_.append( 3*i+1, 3*j+1, TxtJtMinvJTx_ij );
      A_.append( 3*j+1, 3*i+1, TxtJtMinvJTx_ij );
   }
   if( TxtJtMinvJTy_ij != real(0) ) {
      A_.append( 3*i+1, 3*j+2, TxtJtMinvJTy_ij );
      A_.append( 3*j+2, 3*i+1, TxtJtMinvJTy_ij );
   }
   if( TytJtMinvJN_ij  != real(0) ) {
      A_.append( 3*i+2, 3*j,   TytJtMinvJN_ij );
      A_.append( 3*j,   3*i+2, TytJtMinvJN_ij );
   }
   if( TytJtMinvJTx_ij != real(0) ) {
      A_.append( 3*i+2, 3*j+1, TytJtMinvJTx_ij );
      A_.append( 3*j+1, 3*i+2, TytJtMinvJTx_ij );
   }
   if( TytJtMinvJTy_ij != real(0) ) {
      A_.append( 3*i+2, 3*j+2, TytJtMinvJTy_ij );
      A_.append( 3*j+2, 3*i+2, TytJtMinvJTy_ij );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param i TODO
 * \param c_i TODO
 * \param JtMinvJ_ii TODO
 *
 * TODO
 */
template< typename C >  // Type of the configuration
void FrictionConstructor<C>::insertMatrixDiag( size_t i, ConstContactID c_i, const Mat3& JtMinvJ_ii )
{
   const Vec3& n_i ( c_i->getNormal()   );
   const Vec3& tx_i( c_i->getTangentX() );
   const Vec3& ty_i( c_i->getTangentY() );

   const real NtJtMinvJN_ii  ( trans(n_i)  * ( JtMinvJ_ii * n_i  ) );
   const real NtJtMinvJTx_ii ( trans(n_i)  * ( JtMinvJ_ii * tx_i ) );
   const real NtJtMinvJTy_ii ( trans(n_i)  * ( JtMinvJ_ii * ty_i ) );
   const real TxtJtMinvJTx_ii( trans(tx_i) * ( JtMinvJ_ii * tx_i ) );
   const real TxtJtMinvJTy_ii( trans(tx_i) * ( JtMinvJ_ii * ty_i ) );
   const real TytJtMinvJTy_ii( trans(ty_i) * ( JtMinvJ_ii * ty_i ) );

   pe_INTERNAL_ASSERT( NtJtMinvJN_ii   != real(0), "Invalid zero diagonal element encountered." );
   pe_INTERNAL_ASSERT( TxtJtMinvJTx_ii != real(0), "Invalid zero diagonal element encountered." );
   pe_INTERNAL_ASSERT( TytJtMinvJTy_ii != real(0), "Invalid zero diagonal element encountered." );

   A_.append( 3*i, 3*i, NtJtMinvJN_ii );

   if( NtJtMinvJTx_ii != real(0) ) {
      A_.append( 3*i, 3*i+1, NtJtMinvJTx_ii );
      A_.append( 3*i+1, 3*i, NtJtMinvJTx_ii );
   }

   if( NtJtMinvJTy_ii != real(0) ) {
      A_.append( 3*i, 3*i+2, NtJtMinvJTy_ii );
      A_.append( 3*i+2, 3*i, NtJtMinvJTy_ii );
   }

   A_.append( 3*i+1, 3*i+1, TxtJtMinvJTx_ii );

   if( TxtJtMinvJTy_ii != real(0) ) {
      A_.append( 3*i+1, 3*i+2, TxtJtMinvJTy_ii );
      A_.append( 3*i+2, 3*i+1, TxtJtMinvJTy_ii );
   }

   A_.append( 3*i+2, 3*i+2, TytJtMinvJTy_ii );
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
inline void FrictionConstructor<C>::insertVector( size_t i, ConstContactID c_i, const Vec3& Jtb )
{
   b_[3*i  ] = trans(Jtb) * c_i->getNormal() + c_i->getRestitution() * c_i->getNormalRelVel();
   b_[3*i+1] = trans(Jtb) * c_i->getTangentX();
   b_[3*i+2] = trans(Jtb) * c_i->getTangentY();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param contacts TODO
 *
 * TODO
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
inline void FrictionConstructor<C>::postprocess( const Contacts& /*contacts*/ )
{
   // Initialization of the vector of unknowns
   x_ = real(0);
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
