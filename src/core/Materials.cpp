//=================================================================================================
/*!
 *  \file src/core/Materials.cpp
 *  \brief Source file for materials
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2011-2012 Tobias Preclik
 *                2011-2012 Johannes Bleisteiner
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

#include <sstream>
#include <stdexcept>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/math/shims/Square.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

Materials Material::materials_;
MatN Material::corTable_;
MatN Material::csfTable_;
MatN Material::cdfTable_;
bool Material::materialsActivated_( activateMaterials() );
unsigned int Material::anonymousMaterials_ = 0;




//=================================================================================================
//
//  CLASS MATERIALS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Automatic registration of the default materials.
 *
 * \return \a true after the materials have been registered.
 */
bool Material::activateMaterials()
{
   // Registering the default materials
   materials_.push_back( Iron()    );
   materials_.push_back( Copper()  );
   materials_.push_back( Granite() );
   materials_.push_back( Oak()     );
   materials_.push_back( Fir()     );

   // Initializing the coefficients of restitution
   //                       | Iron                   | Copper                 | Granite                | Oak                    | Fir                     |
   const real cor[5][5] = {
                            { static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25) },  // Iron
                            { static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25) },  // Copper
                            { static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25) },  // Granite
                            { static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25) },  // Oak
                            { static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25), static_cast<real>(0.25) }   // Fir
                          };
   corTable_ = cor;

   // Initializing the coefficients of static friction
   //                       | Iron                   | Copper                 | Granite                | Oak                    | Fir                     |
   const real csf[5][5] = {
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Iron
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Copper
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Granite
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Oak
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) }   // Fir
                          };
   csfTable_ = csf;

   // Initializing the coefficients of dynamic friction
   //                       | Iron                   | Copper                 | Granite                | Oak                    | Fir                     |
   const real cdf[5][5] = {
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Iron
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Copper
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Granite
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) },  // Oak
                            { static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20), static_cast<real>(0.20) }   // Fir
                          };
   cdfTable_ = cdf;

   return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  MATERIAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a new custom material.
 * \ingroup materials
 *
 * \param name The name of the custom material.
 * \param density The density of the custom material \f$ (0..\infty) \f$.
 * \param cor The coefficient of restitution of the custom material \f$ [0..1] \f$.
 * \param csf The coefficient of static friction of the custom material \f$ [0..\infty) \f$.
 * \param cdf The coefficient of dynamic friction of the custom material \f$ [0..\infty) \f$.
 * \param poisson The Poisson's ratio of the custom material \f$ [-1..0.5] \f$.
 * \param young The Young's modulus of the custom material \f$ (0..\infty) \f$.
 * \param stiffness The stiffness in normal direction of the material's contact region.
 * \param dampingN The damping coefficient in normal direction of the material's contact region.
 * \param dampingT The damping coefficient in tangential direction of the material's contact region.
 * \return The MaterialID for the new material.
 * \exception std::invalid_argument Invalid material parameter.
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * This function creates the new, custom material \a name with the given properties. The following
 * example illustrates the use of this function:

   \code
   // Creates the material "myMaterial" with the following material properties:
   //  - material density               : 2.54
   //  - coefficient of restitution     : 0.8
   //  - coefficient of static friction : 0.1
   //  - coefficient of dynamic friction: 0.05
   //  - Poisson's ratio                : 0.2
   //  - Young's modulus                : 80
   //  - Contact stiffness              : 100
   //  - dampingN                       : 10
   //  - dampingT                       : 11
   MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05, 0.2, 80, 100, 10, 11 );
   \endcode

 * In case the name of the material is already in use or if any of the coefficients is not in
 * its allowed range, a \a std::invalid_argument exception is thrown.\n
 * Note that the material has to be created on all processes in a MPI parallel simulation. In
 * case this function is called inside an exclusive section, a \a std::runtime_error exception
 * is thrown.
 *
 * The coefficient of restitution is given for self-similar collisions that is collision of bodies
 * made of similar material. The composite coefficient of restitution \f$e_*\f$ is estimated as
 * proposed by Stronge: \f$\frac{e_*^2}{k_*} = \frac{e_1^2}{k_1} + \frac{e_2^2}{k_2}\f$.
 */
MaterialID createMaterial( const std::string& name, real density, real cor,
                           real csf, real cdf, real poisson, real young,
                           real stiffness, real dampingN, real dampingT )
{
   typedef Material M;

   // Checking if the function is called inside an exclusive section
   if( MPISettings::size() > 1 && ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   // Checking the material name
   Materials::const_iterator begin( M::materials_.begin() );
   Materials::const_iterator end  ( M::materials_.end()   );
   for( ; begin!=end; ++begin ) {
      if( begin->getName().compare( name ) == 0 )
         throw std::invalid_argument( "Material of that name already exists!" );
   }

   // Checking the density
   if( density <= real(0) )
      throw std::invalid_argument( "Invalid material density!" );

   // Checking the coefficient of restitution
   if( cor < real(0) || cor > real(1) )
      throw std::invalid_argument( "Invalid coefficient of restitution!" );

   // Checking the coefficient of static friction
   if( csf < real(0) )
      throw std::invalid_argument( "Invalid coefficient of static friction!" );

   // Checking the coefficient of dynamic friction
   if( cdf < real(0) )
      throw std::invalid_argument( "Invalid coefficient of dynamic friction!" );

   // Checking the Poisson's ratio
   if( poisson < real(-1) || poisson > real(0.5) )
      throw std::invalid_argument( "Invalid Poisson's ratio" );

   // Checking the Young's modulus
   if( young <= real(0) )
      throw std::invalid_argument( "Invalid Young's modulus" );

   // Checking the stiffness
   if( stiffness <= real(0) )
      throw std::invalid_argument( "Invalid stiffness" );

   // Checking the damping coefficients
   if( dampingN < real(0) || dampingT < real(0) )
      throw std::invalid_argument( "Invalid damping coefficients" );

   // Registering the new material
   M::materials_.push_back( Material( name, density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT ) );
   const MaterialID mat( M::materials_.size()-1 );

   // Updating the restitution table, the static friction table and the dynamic friction table
   M::corTable_.extend( 1, 1, true );
   M::csfTable_.extend( 1, 1, true );
   M::cdfTable_.extend( 1, 1, true );
   pe_INTERNAL_ASSERT( M::corTable_.rows() == M::corTable_.columns(), "Invalid matrix size" );
   pe_INTERNAL_ASSERT( M::csfTable_.rows() == M::csfTable_.columns(), "Invalid matrix size" );
   pe_INTERNAL_ASSERT( M::cdfTable_.rows() == M::cdfTable_.columns(), "Invalid matrix size" );

   M::corTable_(mat,mat) = cor;
   M::csfTable_(mat,mat) = csf + csf;
   M::cdfTable_(mat,mat) = cdf + cdf;

   for( Materials::size_type i=0; i<mat; ++i ) {
      M::corTable_(mat,i) = M::corTable_(i,mat) = std::sqrt( ( sq( M::materials_[i].getRestitution() ) / M::materials_[i].getStiffness() + sq( cor ) / stiffness ) * M::getStiffness( mat, i ) );
      M::csfTable_(mat,i) = M::csfTable_(i,mat) = M::materials_[i].getStaticFriction()  + csf;
      M::cdfTable_(mat,i) = M::cdfTable_(i,mat) = M::materials_[i].getDynamicFriction() + cdf;
   }

   return mat;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a new anonymous custom material.
// \ingroup materials
//
// \param density The density of the custom material \f$ (0..\infty) \f$.
// \param cor The coefficient of restitution of the custom material \f$ [0..1] \f$.
// \param csf The coefficient of static friction of the custom material \f$ [0..\infty) \f$.
// \param cdf The coefficient of dynamic friction of the custom material \f$ [0..\infty) \f$.
// \param poisson The Poisson's ratio of the custom material \f$ [-1..0.5] \f$.
// \param young The Young's modulus of the custom material \f$ (0..\infty) \f$.
// \param stiffness The stiffness in normal direction of the material's contact region.
// \param dampingN The damping coefficient in normal direction of the material's contact region.
// \param dampingT The damping coefficient in tangential direction of the material's contact region.
// \return The MaterialID for the new material.
// \exception std::invalid_argument Invalid material parameter.
// \exception std::runtime_error Invalid function call inside exclusive section.
//
// This function creates a new, custom material with the given properties. It will be
// named 'Material', followed by an incrementing number.
// Note that the material has to be created on all processes in an MPI parallel simulation. In
// case this function is called inside an exclusive section, a \a std::runtime_error exception
// is thrown.
*/
MaterialID createMaterial( real density, real cor, real csf, real cdf, real poisson, real young,
                           real stiffness, real dampingN, real dampingT )
{
   std::ostringstream sstr;

   do {
      if (Material::anonymousMaterials_ + 1 == 0)
         throw std::runtime_error("Index overflow for anonymous materials");
      sstr.str("");
      sstr << "Material" << ++Material::anonymousMaterials_;
   } while( Material::find( sstr.str() ) != invalid_material );

   return createMaterial( sstr.str(), density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searching for a registered material.
 * \ingroup materials
 *
 * \param name The name of the material.
 * \return The MaterialID of the material if the material is found, \a invalid_material otherwise.
 *
 * The function searches for a registered material with the given name. If the material is found,
 * the corresponding MaterialID is returned. Otherwise, \a invalid_material is returned.
 */
MaterialID Material::find( const std::string& name )
{
   for( Material::SizeType i=0; i<Material::materials_.size(); ++i ) {
      if( Material::materials_[i].getName().compare( name ) == 0 ) {
         return i;
      }
   }
   return invalid_material;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searching for registered materials with a prefix.
// \ingroup materials
//
// \param prefix The prefix common to the names of the materials.
// \return A std::vector object containing the MaterialIDs of all materials found.
//
// The function collects all registered materials with names beginning with the given string.
// Their IDs are assembled in an std::vector object. If no Materials are found, the container is
// empty.
*/
std::vector<MaterialID> Material::findPrefix( const std::string& prefix )
{
   std::vector<MaterialID> results;
   for( Material::SizeType i=0; i<Material::materials_.size(); ++i ) {
      if( Material::materials_[i].getName().compare(0,prefix.size(), prefix ) == 0 ) {
         results.push_back(i);
      }
   }
   return results;
}
//*************************************************************************************************

} // namespace pe
