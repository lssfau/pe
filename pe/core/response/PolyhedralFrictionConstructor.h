//=================================================================================================
/*!
 *  \file pe/core/response/PolyhedralFrictionConstructor.h
 *  \brief Constructs system matrices and right-hand-sides for frictional contact problems.
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

#ifndef _PE_CORE_RESPONSE_POLYHEDRALFRICTIONCONSTRUCTOR_H_
#define _PE_CORE_RESPONSE_POLYHEDRALFRICTIONCONSTRUCTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <pe/math/Matrix3x3.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/problems/LCP.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/system/LCPConfig.h>
#include <pe/system/Precision.h>
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
class PolyhedralFrictionConstructor : private NonCopyable
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
   PolyhedralFrictionConstructor( LCP& lcp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**System construction functions***************************************************************
   /*!\name System construction functions */
   //@{
   inline void allocate( const std::vector<size_t>& nonzeros );
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
 * \param lcp TODO
 */
template< typename C >  // Type of the configuration
inline PolyhedralFrictionConstructor<C>::PolyhedralFrictionConstructor( LCP& lcp )
   : A_( lcp.A_ )  // The system matrix A
   , b_( lcp.b_ )  // The right-hand side vector b
   , x_( lcp.x_ )  // The vector of unknowns x
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
 * TODO: describing the structure of the LCP matrix
 * TODO: more informations
 */
template< typename C >  // Type of the configuration
inline void PolyhedralFrictionConstructor<C>::allocate( const std::vector<size_t>& nonzeros )
{
   using namespace pe::response::lcp;

   const size_t N( nonzeros.size() );
   const size_t n( N * (facets+2) );
   std::vector<size_t> tmp( n );

   for( size_t i=0; i<N; ++i ) {
      tmp[(facets+1)*i] = (facets+1) * nonzeros[i];
      for( size_t j=0; j<facets; ++j )
         tmp[(facets+1)*i+j+1] = (facets+1) * nonzeros[i] + 1;
   }
   for( size_t i=0; i<N; ++i ) {
      tmp[(facets+1)*N+i] = facets+1;
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
void PolyhedralFrictionConstructor<C>::insertMatrixOffdiag( size_t i, size_t j, ConstContactID c_i, ConstContactID c_j, const Mat3& JtMinvJ_ij )
{
   using namespace pe::response::lcp;

   const Vec3& n_i ( c_i->getNormal() );
   const Vec3& n_j ( c_j->getNormal() );
   const size_t offset_i( (facets+1)*i );
   const size_t offset_j( (facets+1)*j );

   Vec3          JtMinvJN_ij( JtMinvJ_ij * n_j );
   VectorN<Vec3> JtMinvJT_ij( facets );
   for( size_t k=0; k<facets; ++k ) {
      JtMinvJT_ij[k] = JtMinvJ_ij * c_j->getTangent(k);
   }

   real NtJtMinvJN_ij( trans(n_i) * JtMinvJN_ij );
   VecN NtJtMinvJT_ij ( facets );
   for( size_t k=0; k<facets; ++k ) {
      NtJtMinvJT_ij[k] = trans(n_i) * JtMinvJT_ij[k];
   }
   VecN TtJtMinvJN_ij( facets );
   for( size_t k=0; k<facets; ++k ) {
      TtJtMinvJN_ij[k] = trans( c_i->getTangent(k) ) * JtMinvJN_ij;
   }
   MatN TtJtMinvJT_ij( facets, facets );
   for( size_t k=0; k<facets; ++k ) {
      for( size_t l=0; l<facets; ++l ) {
         TtJtMinvJT_ij(k, l) = trans( c_i->getTangent(k) ) * JtMinvJT_ij[l];
      }
   }

   // Inserting NtJtMinvJN
   if( NtJtMinvJN_ij != 0 ) {
      A_.append( offset_i, offset_j, NtJtMinvJN_ij );
      A_.append( offset_j, offset_i, NtJtMinvJN_ij );
   }

   // Inserting NtJtMinvJT
   for( size_t k=0; k<facets; ++k ) {
      if( NtJtMinvJT_ij[k] != 0 ) {
         A_.append( offset_i, offset_j+1+k, NtJtMinvJT_ij[k] );
         A_.append( offset_j+1+k, offset_i, NtJtMinvJT_ij[k] );
      }
   }

   // Inserting TtJtMinvJN
   for( size_t k=0; k<facets; ++k ) {
      if( TtJtMinvJN_ij[k] != 0 ) {
         A_.append( offset_i+1+k, offset_j, TtJtMinvJN_ij[k] );
         A_.append( offset_j, offset_i+1+k, TtJtMinvJN_ij[k] );
      }
   }

   // Inserting TtJtMinvJT
   for( size_t k=0; k<facets; ++k ) {
      for( size_t l=0; l<facets; ++l ) {
         if( TtJtMinvJT_ij(k, l) != 0 ) {
            A_.append( offset_i+1+k, offset_j+1+l, TtJtMinvJT_ij(k, l) );
            A_.append( offset_j+1+l, offset_i+1+k, TtJtMinvJT_ij(k, l) );
         }
      }
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
void PolyhedralFrictionConstructor<C>::insertMatrixDiag( size_t i, ConstContactID c_i, const Mat3& JtMinvJ_ii )
{
   using namespace pe::response::lcp;

   const Vec3&  n_i     ( c_i->getNormal() );
   const size_t offset_i( (facets+1)*i );

   Vec3          JtMinvJN_ii( JtMinvJ_ii * n_i );
   VectorN<Vec3> JtMinvJT_ii( facets );
   for( size_t k=0; k<facets; ++k ) {
      JtMinvJT_ii[k] = JtMinvJ_ii * c_i->getTangent(k);
   }

   {
      const real tmp( trans(n_i) * JtMinvJN_ii );
      pe_INTERNAL_ASSERT( tmp != real( 0 ), "Invalid zero diagonal element detected." );
      A_.append( offset_i, offset_i, tmp );
   }

   for( size_t k=0; k<facets; ++k ) {
      const real tmp( trans(n_i) * JtMinvJT_ii[k] );
      if( tmp != real(0) ) {
         A_.append( offset_i, offset_i+1+k, tmp );
         A_.append( offset_i+1+k, offset_i, tmp );
      }
   }

   for( size_t k=0; k<facets; ++k ) {
      {
         const real tmp( trans( c_i->getTangent(k) ) * JtMinvJT_ii[k] );
         pe_INTERNAL_ASSERT( tmp != real(0), "Invalid zero diagonal element detected." );
         A_.append( offset_i+1+k, offset_i+1+k, tmp );
      }

      for( size_t l=k+1; l<facets; ++l ) {
         const real tmp( trans( c_i->getTangent(k) ) * JtMinvJT_ii[l] );
         if( tmp != real(0) ) {
            A_.append( offset_i+1+k, offset_i+1+l, tmp );
            A_.append( offset_i+1+l, offset_i+1+k, tmp );
         }
      }
   }
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
inline void PolyhedralFrictionConstructor<C>::insertVector( size_t i, ConstContactID c_i, const Vec3& Jtb )
{
   using namespace pe::response::lcp;

   const size_t offset_i( (facets+1)*i );

   b_[offset_i] = trans( c_i->getNormal() ) * Jtb + c_i->getRestitution() * c_i->getNormalRelVel();
   for( size_t k=0; k<facets; ++k )
      b_[offset_i+1+k] = trans( c_i->getTangent(k) ) * Jtb;
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
inline void PolyhedralFrictionConstructor<C>::postprocess( const Contacts& contacts )
{
   using namespace pe::response::lcp;

   const size_t N     ( contacts.size() );
   const size_t offset( (facets+1)*N );

   for( size_t i=0; i<N; ++i ) {
      A_.append( offset+i, (facets+1)*i, contacts[i]->getFriction() );
      for( size_t j=0; j<facets; ++j ) {
         A_.append( offset+i, (facets+1)*i+1+j, real( -1 ) );
         A_.append( (facets+1)*i+1+j, offset+i, real( +1 ) );
      }
      b_[offset+i] = real(0);
   }

   x_ = real(0);
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
