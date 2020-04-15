//=================================================================================================
/*!
 *  \file src/core/BodyReader.cpp
 *  \brief Extractor for rigid body parameter files
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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <pe/core/Algorithm.h>
#include <pe/core/BodyReader.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/Link.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/World.h>
#include <pe/math/Constants.h>
#include <pe/math/Matrix3x3.h>
#include <pe/povray/AgateNormal.h>
#include <pe/povray/AgatePigment.h>
#include <pe/povray/Ambient.h>
#include <pe/povray/AreaLight.h>
#include <pe/povray/BozoNormal.h>
#include <pe/povray/BozoPigment.h>
#include <pe/povray/Bumps.h>
#include <pe/povray/Camera.h>
#include <pe/povray/Color.h>
#include <pe/povray/ColorMap.h>
#include <pe/povray/ColorPigment.h>
#include <pe/povray/CustomFinish.h>
#include <pe/povray/CustomNormal.h>
#include <pe/povray/CustomPigment.h>
#include <pe/povray/CustomTexture.h>
#include <pe/povray/Dents.h>
#include <pe/povray/Diffuse.h>
#include <pe/povray/FadeDistance.h>
#include <pe/povray/FadePower.h>
#include <pe/povray/Falloff.h>
#include <pe/povray/Frequency.h>
#include <pe/povray/GraniteNormal.h>
#include <pe/povray/GranitePigment.h>
#include <pe/povray/ImagePigment.h>
#include <pe/povray/Lambda.h>
#include <pe/povray/LayeredTexture.h>
#include <pe/povray/MarbleNormal.h>
#include <pe/povray/MarblePigment.h>
#include <pe/povray/Octaves.h>
#include <pe/povray/Omega.h>
#include <pe/povray/ParallelLight.h>
#include <pe/povray/Phase.h>
#include <pe/povray/Phong.h>
#include <pe/povray/PhongSize.h>
#include <pe/povray/PlainTexture.h>
#include <pe/povray/PointAt.h>
#include <pe/povray/PointLight.h>
#include <pe/povray/RadialPigment.h>
#include <pe/povray/Radius.h>
#include <pe/povray/Reflection.h>
#include <pe/povray/Refraction.h>
#include <pe/povray/Ripples.h>
#include <pe/povray/Rotation.h>
#include <pe/povray/Roughness.h>
#include <pe/povray/Scale.h>
#include <pe/povray/Shadowless.h>
#include <pe/povray/Specular.h>
#include <pe/povray/SpotLight.h>
#include <pe/povray/SpottedNormal.h>
#include <pe/povray/SpottedPigment.h>
#include <pe/povray/Tightness.h>
#include <pe/povray/TiledTexture.h>
#include <pe/povray/Translation.h>
#include <pe/povray/Turbulence.h>
#include <pe/povray/Waves.h>
#include <pe/povray/Wrinkles.h>
#include <pe/povray/Writer.h>
#include <pe/util/InputString.h>
#include <pe/util/Null.h>
#include <pe/util/Time.h>
#include <pe/util/UnsignedValue.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the BodyReader class.
 *
 * The constructor initializes the offset with (0,0,0) and the scaling factors with (1,1,1).
 */
BodyReader::BodyReader()
   : error_()          // Container for all error messages
   , offset_(0)        // Displacement offset for the position of the generated bodies
   , scaling_(1)       // Scaling factors for length, velocity and weight parameters
   , precision_(6)     // The precision for all output operations
   , word_()           // Buffer for extracted keywords
   , commandLine_()    // Line number of the current command
   , lineNumbers_()    // Vector for the line numbers of the input file
   , input_()          // Buffer for the preprocessed rigid body parameters
{
   word_.reserve( 100 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy constructor for the BodyReader class.
 *
 * \param br The body reader to be copied.
 */
BodyReader::BodyReader( const BodyReader& br )
   : error_(br.error_)      // Container for all error messages
   , offset_(br.offset_)    // Displacement offset for the position of the generated bodies
   , scaling_(br.scaling_)  // Scaling factors for length, velocity and weight parameters
   , word_()                // Buffer for extracted keywords
   , commandLine_()         // Line number of the current command
   , lineNumbers_()         // Vector for the line numbers of the input file
   , input_()               // Buffer for the preprocessed rigid body parameters
{
   word_.reserve( 100 );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the BodyReader class.
 */
BodyReader::~BodyReader()
{}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for the BodyReader class.
 *
 * \param br The body reader to be copied.
 * \return Reference to the assigned body reader.
 */
BodyReader& BodyReader::operator=( const BodyReader& br )
{
   if( &br == this ) return *this;

   error_   = br.error_;
   offset_  = br.offset_;
   scaling_ = br.scaling_;

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  INPUT AND OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body setup based on the given parameter file.
 *
 * \param filename The rigid body parameter file.
 * \param povray \a true if a POV-Ray setup if performed, \a false if it is not.
 * \return void
 */
void BodyReader::readFile( const char* const filename, bool povray )
{
   const Domain& domain( theCollisionSystem()->getDomain() );

   ////////////////////////////////////////////////////////
   // Preprocessing the rigid body parameter file stream

   std::ifstream in( filename, std::ifstream::in );
   if( !in.is_open() )
   {
      error_ << "   Error opening parameter input file '" << filename << "' !\n";
      return;
   }

   bool comment( false );
   size_t lineCounter( 0 );
   std::string::size_type pos1, pos2;
   std::string line;
   LabelMap labelMap;

   line.reserve( 100 );

   while( std::getline( in, line ) )
   {
      ++lineCounter;

      // Filtering comments
      if( comment ) {
         if( ( pos1 = line.find( "*/", 0 ) ) != std::string::npos ) {
            line.erase( line.begin(), line.begin()+pos1+2 );
            comment = false;
         }
         else continue;
      }

      if( ( pos1 = line.find( "//", 0 ) ) != std::string::npos ) {
         line.erase( line.begin()+pos1, line.end() );
      }
      if( ( pos1 = line.find( "/*", 0 ) ) != std::string::npos ) {
         if( ( pos2 = line.find( "*/", pos1+2 ) ) != std::string::npos ) {
            line.replace( line.begin()+pos1, line.begin()+pos2+2, " " );
         }
         else {
            line.erase( line.begin()+pos1, line.end() );
            comment = true;
         }
      }

      // Adding whitespaces
      for( pos1=0; ; ++pos1 )
      {
         if( pos1 >= line.size() ) break;

         if( line[pos1] == '{' ) {
            line.replace( pos1, 1, " { " );
            pos1 += 2;
         }
         else if( line[pos1] == '}' ) {
            line.replace( pos1, 1, " } " );
            pos1 += 2;
         }
         else if( line[pos1] == '<' ) {
            line.replace( pos1, 1, " <" );
            ++pos1;
         }
         else if( line[pos1] == '>' ) {
            line.replace( pos1, 1, "> " );
            ++pos1;
         }
         else if( line[pos1] == ';' ) {
            line.replace( pos1, 1, " ; " );
            pos1 += 2;
         }

         else if( line[pos1] == '=' ) {
            line.replace( pos1, 1, " = " );
            pos1 += 2;
         }
      }

      // Adding the line to the input string
      lineNumbers_.push_back( Pair( input_.tellp(), lineCounter ) );
      input_ << line << "\n";
   }

   in.close();


   //////////////////////////////////////////
   // Activating the POV-Ray visualization

   povray::WriterID pov;
   if( povray ) pov = povray::activateWriter();


   //////////////////////////
   // Parameter extraction

   bool fileSet( false ), startSet( false ), endSet( false ), spacingSet( false ),
        backGroundSet( false ), cameraSet( false );

   while( input_ >> word_ )
   {
      /////////////////////////
      // Extracting a sphere

      if( word_.compare( "sphere" ) == 0 )
      {
         SphereParameters sp;
         Error error;

         if( !extractSphere( sp, error, true ) ) {
            error_ << " Sphere (line " << sp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !sp.materialSet ) {
            error_ << " Sphere (line " << sp.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( sp.center ) )
         {
            // Creating a new sphere
            SphereID sphere = createSphere( sp.id, sp.center, sp.radius, sp.material, sp.visible );

            // Registering label
            if( labelMap.find( sp.label ) != labelMap.end() ) {
               error_ << " Sphere label \"" << sp.label << "\" (line " << sp.line << ") is ambiguous.\n";
            }
            else if( !sp.label.empty() ) labelMap[ sp.label ] = sphere;

            // Configuring the sphere
            sphere->translate( sp.translation );
            sphere->setLinearVel( sp.linear );
            sphere->setAngularVel( sp.angular );
            sphere->setFixed( sp.fixed );

            // Rotating the sphere
            const Rotations::const_iterator rend( sp.rotations.end() );
            for( Rotations::const_iterator rot=sp.rotations.begin(); rot!=rend; ++rot )
               sphere->rotate( *rot );

            if( povray && sp.textureSet ) pov->setTexture( sphere, sp.texture );
         }
      }


      //////////////////////
      // Extracting a box

      else if( word_.compare( "box" ) == 0 )
      {
         BoxParameters bp;
         Error error;

         if( !extractBox( bp, error, true ) ) {
            error_ << " Box (line " << bp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !bp.materialSet ) {
            error_ << " Box (line " << bp.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( bp.center ) )
         {
            // Creating a new box
            BoxID box = createBox( bp.id, bp.center, bp.lengths, bp.material, bp.visible );

            // Registering label
            if( labelMap.find( bp.label ) != labelMap.end() ) {
               error_ << " Box label \"" << bp.label << "\" (line " << bp.line << ") is ambiguous.\n";
            }
            else if( !bp.label.empty() ) labelMap[ bp.label ] = box;

            // Configuring the box
            box->translate( bp.translation );
            box->setLinearVel( bp.linear );
            box->setAngularVel( bp.angular );
            box->setFixed( bp.fixed );

            // Rotating the box
            const Rotations::const_iterator rend( bp.rotations.end() );
            for( Rotations::const_iterator rot=bp.rotations.begin(); rot!=rend; ++rot )
               box->rotate( *rot );

            if( povray && bp.textureSet ) pov->setTexture( box, bp.texture );
         }
      }


      //////////////////////////
      // Extracting a capsule

      else if( word_.compare( "capsule" ) == 0 )
      {
         CapsuleParameters cp;
         Error error;

         if( !extractCapsule( cp, error, true ) ) {
            error_ << " Capsule (line " << cp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !cp.materialSet ) {
            error_ << " Capsule (line " << cp.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( cp.center ) )
         {
            // Creating a new capsule
            CapsuleID capsule = createCapsule( cp.id, cp.center, cp.radius,
                                               cp.length, cp.material, cp.visible );

            // Registering label
            if( labelMap.find( cp.label ) != labelMap.end() ) {
               error_ << " Capsule label \"" << cp.label << "\" (line " << cp.line << ") is ambiguous.\n";
            }
            else if( !cp.label.empty() ) labelMap[ cp.label ] = capsule;

            // Configuring the capsule
            capsule->translate( cp.translation );
            capsule->setLinearVel( cp.linear );
            capsule->setAngularVel( cp.angular );
            capsule->setFixed( cp.fixed );

            // Rotating the capsule
            const Rotations::const_iterator rend( cp.rotations.end() );
            for( Rotations::const_iterator rot=cp.rotations.begin(); rot!=rend; ++rot )
               capsule->rotate( *rot );

            if( povray && cp.textureSet ) pov->setTexture( capsule, cp.texture );
         }
      }


      //////////////////////////
      // Extracting a cylinder

      else if( word_.compare( "cylinder" ) == 0 )
      {
         CylinderParameters cp;
         Error error;

         if( !extractCylinder( cp, error, true ) ) {
            error_ << " Cylinder (line " << cp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !cp.materialSet ) {
            error_ << " Cylinder (line " << cp.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( cp.center ) )
         {
            // Creating a new cylinder
            CylinderID cylinder = createCylinder( cp.id, cp.center, cp.radius,
                                                  cp.length, cp.material, cp.visible );

            // Registering label
            if( labelMap.find( cp.label ) != labelMap.end() ) {
               error_ << " Cylinder label \"" << cp.label << "\" (line " << cp.line << ") is ambiguous.\n";
            }
            else if( !cp.label.empty() ) labelMap[ cp.label ] = cylinder;

            // Configuring the cylinder
            cylinder->translate( cp.translation );
            cylinder->setLinearVel( cp.linear );
            cylinder->setAngularVel( cp.angular );
            cylinder->setFixed( cp.fixed );

            // Rotating the cylinder
            const Rotations::const_iterator rend( cp.rotations.end() );
            for( Rotations::const_iterator rot=cp.rotations.begin(); rot!=rend; ++rot )
               cylinder->rotate( *rot );

            if( povray && cp.textureSet ) pov->setTexture( cylinder, cp.texture );
         }
      }


      ////////////////////////
      // Extracting a plane

      else if( word_.compare( "plane" ) == 0 )
      {
         PlaneParameters pp;
         Error error;

         if( !extractPlane( pp, error ) ) {
            error_ << " Plane (line " << pp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !pp.materialSet ) {
            error_ << " Plane (line " << pp.line << ") could not be created: no material information available!\n";
         }
         else
         {
            pe_GLOBAL_SECTION
            {
               // Creating a new plane
               PlaneID plane = createPlane( pp.id, pp.normal, pp.displacement, pp.material, pp.visible );

               // Registering label
               if( labelMap.find( pp.label ) != labelMap.end() ) {
                  error_ << " Plane label \"" << pp.label << "\" (line " << pp.line << ") is ambiguous.\n";
               }
               else if( !pp.label.empty() ) labelMap[ pp.label ] = plane;

               // Configuring the plane
               plane->translate( pp.translation );

               // Rotating the plane
               const Rotations::const_iterator rend( pp.rotations.end() );
               for( Rotations::const_iterator rot=pp.rotations.begin(); rot!=rend; ++rot )
                  plane->rotate( *rot );

               if( povray && pp.textureSet ) pov->setTexture( plane, pp.texture );
            }
         }
      }


      //////////////////////////////
      // Extracting a tetrasphere

      else if( word_.compare( "tetrasphere" ) == 0 )
      {
         SphereParameters sp;
         Error error;

         if( !extractSphere( sp, error, true ) ) {
            error_ << " Tetrasphere (line " << sp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !sp.materialSet ) {
            error_ << " Tetrasphere (line " << sp.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( sp.center ) )
         {
            // Creating a new tetrasphere
            UnionID u = createTetrasphere( sp.id, sp.center, sp.radius, sp.material, sp.visible );

            // Registering label
            if( labelMap.find( sp.label ) != labelMap.end() ) {
               error_ << " Tetrasphere label \"" << sp.label << "\" (line " << sp.line << ") is ambiguous.\n";
            }
            else if( !sp.label.empty() ) labelMap[ sp.label ] = u;

            // Configuring the tetrasphere
            u->translate( sp.translation );
            u->setLinearVel( sp.linear );
            u->setAngularVel( sp.angular );
            u->setFixed( sp.fixed );

            // Rotating the sphere
            const Rotations::const_iterator rend( sp.rotations.end() );
            for( Rotations::const_iterator rot=sp.rotations.begin(); rot!=rend; ++rot )
               u->rotate( *rot );

            if( povray && sp.textureSet ) pov->setTexture( u, sp.texture );
         }
      }


      ///////////////////////////////
      // Extracting an agglomerate

      else if( word_.compare( "agglomerate" ) == 0 )
      {
         AgglomerateParameters ap;
         Error error;

         if( !extractAgglomerate( ap, error ) ) {
            error_ << " Agglomerate (line " << ap.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( !ap.materialSet ) {
            error_ << " Agglomerate (line " << ap.line << ") could not be created: no material information available!\n";
         }
         else if( domain.ownsPoint( ap.center ) )
         {
            // Creating a new agglomerate
            UnionID agglomerate = createAgglomerate( ap.id, ap.center, ap.radius, ap.material,
                                                     ap.number, ap.threshold, ap.visible );

            // Registering label
            if( labelMap.find( ap.label ) != labelMap.end() ) {
               error_ << " Agglomerate label \"" << ap.label << "\" (line " << ap.line << ") is ambiguous.\n";
            }
            else if( !ap.label.empty() ) labelMap[ ap.label ] = agglomerate;

            // Configuring the agglomerate
            agglomerate->translate( ap.translation );
            agglomerate->setLinearVel( ap.linear );
            agglomerate->setAngularVel( ap.angular );
            agglomerate->setFixed( ap.fixed );

            const Rotations::const_iterator rend( ap.rotations.end() );
            for( Rotations::const_iterator rot=ap.rotations.begin(); rot!=rend; ++rot )
               agglomerate->rotate( *rot );
         }
      }


      ////////////////////////
      // Extracting a union

      else if( word_.compare( "union" ) == 0 )
      {
         UnionParameters up;
         Error error;

         if( !extractUnion( up, error ) ) {
            error_ << " Union (line " << up.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( MPISettings::size() > 1 && !up.planes.empty() ) {
            error_ << " Union (line " << up.line << "): Cannot create planes inside a union in MPI parallel environment!\n";
         }
         else
         {
            // Creating a new union
            pe_CREATE_UNION( u, up.id )
            {
               // Registering label
               if( labelMap.find( up.label ) != labelMap.end() ) {
                  error_ << " Union label \"" << up.label << "\" (line " << up.line << ") is ambiguous.\n";
               }
               else if( !up.label.empty() ) labelMap[ up.label ] = u;

               // Setup of all specified spheres
               for( SphereParameterVector::const_iterator s=up.spheres.begin(); s!=up.spheres.end(); ++s )
               {
                  // Creating a new sphere within the union
                  SphereID sphere = createSphere( s->id, s->center, s->radius, s->material, s->visible );

                  // Registering label
                  if( labelMap.find( s->label ) != labelMap.end() ) {
                     error_ << " Sphere label \"" << s->label << "\" (line " << s->line << ") is ambiguous.\n";
                  }
                  else if( !s->label.empty() ) labelMap[ s->label ] = sphere;

                  // Translating the sphere within the union
                  sphere->translate( s->translation );

                  // Rotating the sphere within the union
                  const Rotations::const_iterator rend( s->rotations.end() );
                  for( Rotations::const_iterator rot=s->rotations.begin(); rot!=rend; ++rot )
                     sphere->rotate( *rot );

                  // Setting the POV-Ray texture
                  if( povray ) {
                     if( s->textureSet ) pov->setTexture( sphere, s->texture );
                     else if( up.textureSet ) pov->setTexture( sphere, up.texture );
                  }
               }

               // Setup of all specified boxes
               for( BoxParameterVector::const_iterator b=up.boxes.begin(); b!=up.boxes.end(); ++b )
               {
                  // Creating a new box within the union
                  BoxID box = createBox( b->id, b->center, b->lengths, b->material, b->visible );

                  // Registering label
                  if( labelMap.find( b->label ) != labelMap.end() ) {
                     error_ << " Box label \"" << b->label << "\" (line " << b->line << ") is ambiguous.\n";
                  }
                  else if( !b->label.empty() ) labelMap[ b->label ] = box;

                  // Translating the box within the union
                  box->translate( b->translation );

                  // Rotating the box within the union
                  const Rotations::const_iterator rend( b->rotations.end() );
                  for( Rotations::const_iterator rot=b->rotations.begin(); rot!=rend; ++rot )
                     box->rotate( *rot );

                  // Setting the POV-Ray texture
                  if( povray ) {
                     if( b->textureSet ) pov->setTexture( box, b->texture );
                     else if( up.textureSet ) pov->setTexture( box, up.texture );
                  }
               }

               // Setup of all specified capsules
               for( CapsuleParameterVector::const_iterator c=up.capsules.begin(); c!=up.capsules.end(); ++c )
               {
                  // Creating a new capsule within the union
                  CapsuleID capsule = createCapsule( c->id, c->center, c->radius,
                                                   c->length, c->material, c->visible );

                  // Registering label
                  if( labelMap.find( c->label ) != labelMap.end() ) {
                     error_ << " Capsule label \"" << c->label << "\" (line " << c->line << ") is ambiguous.\n";
                  }
                  else if( !c->label.empty() ) labelMap[ c->label ] = capsule;

                  // Translating the capsule within the union
                  capsule->translate( c->translation );

                  // Rotating the capsule within the union
                  const Rotations::const_iterator rend( c->rotations.end() );
                  for( Rotations::const_iterator rot=c->rotations.begin(); rot!=rend; ++rot )
                     capsule->rotate( *rot );

                  // Setting the POV-Ray texture
                  if( povray ) {
                     if( c->textureSet ) pov->setTexture( capsule, c->texture );
                     else if( up.textureSet ) pov->setTexture( capsule, up.texture );
                  }
               }

               // Setup of all specified cylinders
               for( CylinderParameterVector::const_iterator c=up.cylinders.begin(); c!=up.cylinders.end(); ++c )
               {
                  // Creating a new cylinder within the union
                  CylinderID cylinder = createCylinder( c->id, c->center, c->radius,
                                                      c->length, c->material, c->visible );

                  // Registering label
                  if( labelMap.find( c->label ) != labelMap.end() ) {
                     error_ << " Cylinder label \"" << c->label << "\" (line " << c->line << ") is ambiguous.\n";
                  }
                  else if( !c->label.empty() ) labelMap[ c->label ] = cylinder;

                  // Translating the cylinder within the union
                  cylinder->translate( c->translation );

                  // Rotating the cylinder within the union
                  const Rotations::const_iterator rend( c->rotations.end() );
                  for( Rotations::const_iterator rot=c->rotations.begin(); rot!=rend; ++rot )
                     cylinder->rotate( *rot );

                  // Setting the POV-Ray texture
                  if( povray ) {
                     if( c->textureSet ) pov->setTexture( cylinder, c->texture );
                     else if( up.textureSet ) pov->setTexture( cylinder, up.texture );
                  }
               }

               // Setting up all specified planes
               for( PlaneParameterVector::const_iterator p=up.planes.begin(); p!=up.planes.end(); ++p )
               {
                  // Creating a new plane within the union
                  PlaneID plane = createPlane( p->id, p->normal, p->displacement,
                                             p->material, p->visible );

                  // Registering label
                  if( labelMap.find( p->label ) != labelMap.end() ) {
                     error_ << " Plane label \"" << p->label << "\" (line " << p->line << ") is ambiguous.\n";
                  }
                  else if( !p->label.empty() ) labelMap[ p->label ] = plane;

                  // Rotating the plane within the union
                  const Rotations::const_iterator rend( p->rotations.end() );
                  for( Rotations::const_iterator rot=p->rotations.begin(); rot!=rend; ++rot )
                     plane->rotate( *rot );

                  // Setting the POV-Ray texture
                  if( povray ) {
                     if( p->textureSet ) pov->setTexture( plane, p->texture );
                     else if( up.textureSet ) pov->setTexture( plane, up.texture );
                  }
               }

               // Setting up all specified links
               const Union::CastIterator<Sphere>   sphereBegin  ( u->begin<Sphere>()   );
               const Union::CastIterator<Sphere>   sphereEnd    ( u->end<Sphere>()     );
               const Union::CastIterator<Box>      boxBegin     ( u->begin<Box>()      );
               const Union::CastIterator<Box>      boxEnd       ( u->end<Box>()        );
               const Union::CastIterator<Capsule>  capsuleBegin ( u->begin<Capsule>()  );
               const Union::CastIterator<Capsule>  capsuleEnd   ( u->end<Capsule>()    );
               const Union::CastIterator<Cylinder> cylinderBegin( u->begin<Cylinder>() );
               const Union::CastIterator<Cylinder> cylinderEnd  ( u->end<Cylinder>()   );

               for( LinkParameterVector::const_iterator link=up.links.begin(); link!=up.links.end(); ++link )
               {
                  BodyID body1( 0 ), body2( 0 );

                  // Determining the first connected rigid body
                  LabelMap::const_iterator it1( labelMap.find( link->body1 ) );
                  if( it1 != labelMap.end() && it1->second->getSuperBody() == u )
                     body1 = it1->second;

                  // Determining the second connected rigid body
                  LabelMap::const_iterator it2( labelMap.find( link->body2 ) );
                  if( it2 != labelMap.end() && it2->second->getSuperBody() == u )
                     body2 = it2->second;

                  if( body1 != 0 && body2 != 0 ) {
                     // Creating a new link between the two rigid bodies
                     createLink( u, link->id, body1, body2 );
                  }
                  else
                     error_ << " Not all body references could be resolved.\n";
               }

               // Configuring the union
               if( up.centerSet ) u->setPosition( up.center );
               u->translate( up.translation );
               u->setLinearVel( up.linear );
               u->setAngularVel( up.angular );
               u->setFixed( up.fixed );
               u->setVisible( up.visible );

               const Rotations::const_iterator rend( up.rotations.end() );
               for( Rotations::const_iterator rot=up.rotations.begin(); rot!=rend; ++rot )
                  u->rotate( *rot );
            }
         }
      }


      ///////////////////////////////////////
      // Extracting a spring-damper system

      else if( word_.compare( "spring" ) == 0 )
      {
         SpringParameters sp;
         Error error;

         if( !extractSpring( sp, error ) ) {
            error_ << " Spring (line " << sp.line << ") could not be created!\n";
            error_.append( error );
         }
         else if( MPISettings::size() > 1 ) {
            error_ << " Cannot create spring in MPI parallel environment (line " << sp.line << ")!\n";
         }
         else {
            LabelMap::const_iterator it1( labelMap.find( sp.body1 ) );
            LabelMap::const_iterator it2( labelMap.find( sp.body2 ) );
            if( it1 == labelMap.end() )
               error_ << "Spring body1 reference \"" << sp.body1 << "\" (line " << sp.line << ") cannot be resolved.\n";
            else if( it2 == labelMap.end() )
               error_ << "Spring body2 reference \"" << sp.body2 << "\" (line " << sp.line << ") cannot be resolved.\n";
            else {
               // Creating a new spring
               SpringID spring = attachSpring( it1->second, sp.anchor1, it2->second, sp.anchor2,
                                               sp.stiffness, sp.damping, sp.visible );

               // Setting the rest length of the spring
               if( sp.length > real(0) ) spring->setLength( sp.length );
            }
         }
      }


      ///////////////////////////
      // Extracting a material

      else if( word_.compare( "material" ) == 0 )
      {
         MaterialID material;
         extractMaterial( material, error_ );
      }


      //////////////////////////////////////////
      // Extracting a POV-Ray include command

      else if( word_.compare( "#include" ) == 0 )
      {
         InputString string;

         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the include file
         if( !(input_ >> string) ) {
            error_ << "   Line " << commandLine_ << ": Invalid '#include' command!\n";
            input_.clear();
         }
         else if( povray ) pov->include( string.str() );
      }


      //////////////////////////////////////////
      // Extracting a POV-Ray declare command

      else if( word_.compare( "#declare" ) == 0 )
      {
         std::string identifier;
         EqualSign equalsign;

         // Calculating the line number
         commandLine_ = getLineNumber();

         if( !(input_ >> identifier >> equalsign >> word_) ) {
            error_ << "   Line " << commandLine_ << ": Invalid '#declare' command!\n";
            input_.clear();
            continue;
         }

         // Declaring a color value
         else if( word_.compare( "color" ) == 0 )
         {
            povray::Color color;
            Semicolon semicolon;

            if( !(input_ >> color >> semicolon) ) {
               error_ << "   Line " << commandLine_ << ": Invalid 'color' declaration!\n";
               input_.clear();
            }
            else if( povray ) pov->declare( identifier, color );
         }

         // Declaring a color map
         else if( word_.compare( "color_map" ) == 0 )
         {
            povray::ColorMap colormap;
            LeadingBracket leadingbracket;
            TrailingBracket trailingbracket;

            if( !(input_ >> leadingbracket >> colormap >> trailingbracket ) ) {
               error_ << "   Line " << commandLine_ << ": Invalid 'color_map' declaration!\n";
               input_.clear();
               skipBlock();
            }
            else if( povray ) pov->declare( identifier, colormap );
         }

         // Declaring a POV-Ray pigment
         else if( word_.compare( "pigment" ) == 0 )
         {
            povray::Pigment pigment;
            Error error;

            if( !extractPigment( pigment, error ) ) {
               error_.append( error );
            }
            else if( povray ) {
               pov->declare( identifier, pigment );
            }
         }

         // Declaring a POV-Ray finish
         else if( word_.compare( "finish" ) == 0 )
         {
            povray::Finish finish;
            Error error;

            if( !extractFinish( finish, error ) ) {
               error_.append( error );
            }
            else if( povray ) {
               pov->declare( identifier, finish );
            }
         }

         // Declaring a POV-Ray normal
         else if( word_.compare( "normal" ) == 0 )
         {
            povray::Normal normal;
            Error error;

            if( !extractNormal( normal, error ) ) {
               error_.append( error );
            }
            else if( povray ) {
               pov->declare( identifier, normal );
            }
         }

         // Declaring a POV-Ray texture
         else if( word_.compare( "texture" ) == 0 )
         {
            povray::Texture texture;
            Error error;

            if( !extractTexture( texture, error ) ) {
               error_.append( error );
            }
            else if( povray ) {
               pov->declare( identifier, texture );
            }
         }
      }


      ///////////////////////////////////////////
      // Extracting the POV-Ray path/file name

      else if( word_.compare( "file" ) == 0 )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate file command
         if( fileSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'file' command!\n";
         }
         else fileSet = true;

         // Extracting the file name
         InputString string;

         if( !(input_ >> string) ) {
            error_ << "   Line " << commandLine_ << ": Invalid 'file' command!\n";
            input_.clear();
         }

         // Checking the file name
         else {
            const std::string file( string.str() );

            if( ( pos1 = file.find_first_of( '%', 0 ) ) == std::string::npos ||
                  file.find_last_of( '%', std::string::npos ) != pos1          ||
                  ( ( pos2 = file.find_last_of( '/', std::string::npos ) ) != std::string::npos && pos1 < pos2 ) ) {
               error_ << "   Line " << commandLine_ << ": Invalid file name!\n";
               input_.clear();
            }
            else if( povray ) pov->setFilename( file );
         }
      }


      //////////////////////////////////
      // Extracting the POV-Ray start

      else if( word_.compare( "start" ) == 0 )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate start command
         if( startSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'start' command!\n";
         }
         else startSet = true;

         // Extracting the start
         UnsignedValue<unsigned int> start;

         if( !(input_ >> start) ) {
            error_ << "   Line " << commandLine_ << ": Invalid 'start' command!\n";
            input_.clear();
         }
         else if( povray ) pov->setStart( start );
      }


      ////////////////////////////////
      // Extracting the POV-Ray end

      else if( word_.compare( "end" ) == 0 )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate end command
         if( endSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'end' command!\n";
         }
         else endSet = true;

         // Extracting the end
         UnsignedValue<unsigned int> end;

         if( !(input_ >> end) ) {
            error_ << "   Line " << commandLine_ << ": Invalid 'end' command!\n";
            input_.clear();
         }
         else if( povray ) pov->setEnd( end );
      }


      ////////////////////////////////////
      // Extracting the POV-Ray spacing

      else if( word_.compare( "spacing" ) == 0 )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate spacing command
         if( spacingSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'spacing' command!\n";
         }
         else spacingSet = true;

         // Extracting the spacing
         UnsignedValue<unsigned int> spacing;

         if( !(input_ >> spacing) || spacing == 0 ) {
            error_ << "   Line " << commandLine_ << ": Invalid 'spacing' command!\n";
            input_.clear();
         }
         else if( povray ) pov->setSpacing( spacing );
      }


      /////////////////////////////////////////////////////
      // Extracting the POV-Ray visualization background

      else if( word_.compare( "background" ) == 0 )
      {
         LeadingBracket leadingbracket;
         TrailingBracket trailingbracket;

         // After the 'background' keyword, a leading bracket is expected. If this bracket
         // is missing, the background extraction is not started and it is assumed that no
         // background section is following.
         if( !(input_ >> leadingbracket) ) {
            input_.clear();
            continue;
         }

         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate file command
         if( backGroundSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'background' command!\n";
         }
         else backGroundSet = true;

         // Extracting the background color
         povray::Color color;

         if( !(input_ >> color >> trailingbracket) ) {
            error_ << "   Line " << commandLine_ << ": Invalid 'background' command!\n";
            input_.clear();
         }
         else if( povray ) {
            pov->setBackground( color );
         }
      }


      ///////////////////////////////////
      // Extracting the POV-Ray camera

      else if( word_.compare( "camera" ) == 0 )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Detecting a duplicate camera command
         if( cameraSet ) {
            error_ << "   Line " << commandLine_ << ": Duplicate 'camera' command!\n";
         }
         else cameraSet = true;

         // Extracting the camera
         Error error;

         if( !extractCamera( error ) ) {
            error_.append( error );
         }
      }


      //////////////////////////////////////
      // Extracting a POV-Ray lightsource

      else if( word_.compare( "light_source" ) == 0 )
      {
         povray::LightSource lightsource;
         Error error;

         if( !extractLightSource( lightsource, error ) ) {
            error_.append( error );
         }
         else if( povray ) {
            pov->addLightSource( lightsource );
         }
      }
   }


   ///////////////////
   // Finalizations

   input_.str( "" );
   lineNumbers_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a sphere primitive.
 *
 * \param sphere The sphere parameters to be extracted.
 * \param error The error container for possible error messages.
 * \param superordinate \a true if the sphere is not contained within a union, \a false if it is.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractSphere( SphereParameters& sphere, Error& error, bool superordinate )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), centerSet( false ), radiusSet( false ), linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the sphere section
   sphere.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << sphere.line << ": Invalid 'sphere' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the sphere parameters
   while( input_ >> word_ )
   {
      //Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific sphere ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the sphere ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.id = id;
      }

      // Setting the sphere label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the sphere label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.label = label;
      }

      // Setting the center of the sphere
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else centerSet = true;

         // Extracting the center
         if( !(input_ >> sphere.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.center = sphere.center*scaling_[length] + offset_;
      }

      // Setting the radius of the sphere
      else if( word_.compare( "radius" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate radius command
         if( radiusSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'radius' command!\n";
         }
         else radiusSet = true;

         // Extracting the radius
         if( !(input_ >> sphere.radius) || sphere.radius <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'radius' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.radius *= scaling_[length];
      }

      // Setting the material of the sphere
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( sphere.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else sphere.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( sphere.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( sphere.material, error );
      }

      // Setting the linear velocity of the sphere
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the sphere
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'linear' command is not allowed for subordiante spheres!\n";
         }

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> sphere.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the sphere
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the sphere
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'angular' command is not allowed for subordiante spheres!\n";
         }

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> sphere.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.angular *= scaling_[velocity];
      }

      // Translation of the sphere
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.translation += translation;
      }

      // Rotating the sphere
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else sphere.rotations.push_back( rotation );
      }

      // Fixing the sphere's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the sphere
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'fixed' command is not allowed for subordiante spheres!\n";
         }

         // Detecting a duplicate fixed command
         if( sphere.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         sphere.fixed = true;
      }

      // Making the sphere invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !sphere.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         sphere.visible = false;
      }

      // Setting the POV-Ray texture of the sphere
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the sphere
         if( sphere.textureSet )
            sphere.texture = povray::LayeredTexture( sphere.texture, texture );
         else sphere.texture = texture;

         sphere.textureSet = true;
      }

      // Ending the 'sphere' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !centerSet || !radiusSet ) {
            error << "   Line " << sphere.line << ": Incomplete 'sphere' command!\n"
                   << "    Necessary sphere options are: 'id', 'center' and 'radius'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'sphere' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'sphere' section starting in line " << sphere.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a box primitive.
 *
 * \param box The box parameters to be extracted.
 * \param error The error container for possible error messages.
 * \param superordinate \a true if the box is not contained within a union, \a false if it is.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractBox( BoxParameters& box, Error& error, bool superordinate )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), centerSet( false ), lengthsSet( false ), linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the box section
   box.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << box.line << ": Invalid 'box' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the box parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific box ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the box ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.id = id;
      }

      // Setting the box label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the box label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.label = label;
      }

      // Setting the center of the box
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else centerSet = true;

         // Extracting the center
         if( !(input_ >> box.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.center = box.center*scaling_[length] + offset_;
      }

      // Setting the side lengths of the box
      else if( word_.compare( "lengths" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate lengths command
         if( lengthsSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'lengths' command!\n";
         }
         else lengthsSet = true;

         // Extracting the side lengths
         if( !(input_ >> box.lengths) ) {
            error << "   Line " << commandLine_ << ": Invalid 'lengths' command!\n";
            errorMode = true;
            input_.clear();
         }
         else if( box.lengths[0] <= real(0) || box.lengths[1] <= real(0) || box.lengths[2] <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid box side lengths!\n";
         }
         else box.lengths *= scaling_[length];
      }

      // Setting the material of the box
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( box.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else box.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( box.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( box.material, error );
      }

      // Setting the linear velocity of the box
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the box
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'linear' command is not allowed for subordiante boxes!\n";
         }

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> box.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the box
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the box
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'angular' command is not allowed for subordiante boxes!\n";
         }

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> box.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.angular *= scaling_[velocity];
      }

      // Translation of the box
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.translation += translation;
      }

      // Rotating the box
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else box.rotations.push_back( rotation );
      }

      // Fixing the box's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the box
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'fixed' command is not allowed for subordiante boxes!\n";
         }

         // Detecting a duplicate fixed command
         if( box.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         box.fixed = true;
      }

      // Making the box invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !box.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         box.visible = false;
      }

      // Setting the POV-Ray texture of the box
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the box
         if( box.textureSet )
            box.texture = povray::LayeredTexture( box.texture, texture );
         else box.texture = texture;

         box.textureSet = true;
      }

      // Ending the 'box' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !centerSet || !lengthsSet ) {
            error << "   Line " << box.line << ": Incomplete 'box' command!\n"
                  << "     Necessary box options are: 'id', 'center' and 'lengths'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'box' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'box' section starting in line " << box.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a capsule primitive.
 *
 * \param capsule The capsule parameters to be extracted.
 * \param error The error container for possible error messages.
 * \param superordinate \a true if the capsule is not contained within a union, \a false if it is.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractCapsule( CapsuleParameters& capsule, Error& error, bool superordinate )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), centerSet( false ), radiusSet( false ), lengthSet( false ),
        linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the capsule section
   capsule.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << capsule.line << ": Invalid 'capsule' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the capsule parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific capsule ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the capsule ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.id = id;
      }

      // Setting the capsule label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the capsule label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.label = label;
      }

      // Setting the center of the capsule
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else centerSet = true;

         // Extracting the center
         if( !(input_ >> capsule.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.center = capsule.center*scaling_[length] + offset_;
      }

      // Setting the radius of the capsule
      else if( word_.compare( "radius" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate radius command
         if( radiusSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'radius' command!\n";
         }
         else radiusSet = true;

         // Extracting the radius
         if( !(input_ >> capsule.radius) || capsule.radius <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'radius' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.radius *= scaling_[length];
      }

      // Setting the length of the capsule
      else if( word_.compare( "length" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate radius command
         if( lengthSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'length' command!\n";
         }
         else lengthSet = true;

         // Extracting the radius
         if( !(input_ >> capsule.length) || capsule.length <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'length' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.length *= scaling_[length];
      }

      // Setting the material of the capsule
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( capsule.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else capsule.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( capsule.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( capsule.material, error );
      }

      // Setting the linear velocity of the capsule
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the capsule
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'linear' command is not allowed for subordiante capsules!\n";
         }

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> capsule.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the capsule
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the capsule
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'angular' command is not allowed for subordiante capsules!\n";
         }

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> capsule.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.angular *= scaling_[velocity];
      }

      // Translation of the capsule
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.translation += translation;
      }

      // Rotating the capsule
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else capsule.rotations.push_back( rotation );
      }

      // Fixing the capsule's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the capsule
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'fixed' command is not allowed for subordiante capsules!\n";
         }

         // Detecting a duplicate fixed command
         if( capsule.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         capsule.fixed = true;
      }

      // Making the capsule invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !capsule.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         capsule.visible = false;
      }

      // Setting the POV-Ray texture of the capsule
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the capsule
         if( capsule.textureSet )
            capsule.texture = povray::LayeredTexture( capsule.texture, texture );
         else capsule.texture = texture;

         capsule.textureSet = true;
      }

      // Ending the 'capsule' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !centerSet || !radiusSet || !lengthSet ) {
            error << "   Line " << capsule.line << ": Incomplete 'capsule' command!\n"
                  << "     Necessary capsule options are: 'id', 'center', 'radius' and 'length'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'capsule' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'capsule' section starting in line " << capsule.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a cylinder primitive.
 *
 * \param cylinder The cylinder parameters to be extracted.
 * \param error The error container for possible error messages.
 * \param superordinate \a true if the cylinder is not contained within a union, \a false if it is.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractCylinder( CylinderParameters& cylinder, Error& error, bool superordinate )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), centerSet( false ), radiusSet( false ), lengthSet( false ),
        linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the cylinder section
   cylinder.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << cylinder.line << ": Invalid 'cylinder' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the cylinder parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific cylinder ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the cylinder ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.id = id;
      }

      // Setting the cylinder label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the cylinder label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.label = label;
      }

      // Setting the center of the cylinder
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else centerSet = true;

         // Extracting the center
         if( !(input_ >> cylinder.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.center = cylinder.center*scaling_[length] + offset_;
      }

      // Setting the radius of the cylinder
      else if( word_.compare( "radius" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate radius command
         if( radiusSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'radius' command!\n";
         }
         else radiusSet = true;

         // Extracting the radius
         if( !(input_ >> cylinder.radius) || cylinder.radius <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'radius' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.radius *= scaling_[length];
      }

      // Setting the length of the cylinder
      else if( word_.compare( "length" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate radius command
         if( lengthSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'length' command!\n";
         }
         else lengthSet = true;

         // Extracting the radius
         if( !(input_ >> cylinder.length) || cylinder.length <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'length' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.length *= scaling_[length];
      }

      // Setting the material of the cylinder
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( cylinder.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else cylinder.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( cylinder.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( cylinder.material, error );
      }

      // Setting the linear velocity of the cylinder
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the cylinder
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'linear' command is not allowed for subordiante cylinders!\n";
         }

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> cylinder.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the cylinder
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the cylinder
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'angular' command is not allowed for subordiante cylinders!\n";
         }

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> cylinder.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.angular *= scaling_[velocity];
      }

      // Translation of the cylinder
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.translation += translation;
      }

      // Rotating the cylinder
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else cylinder.rotations.push_back( rotation );
      }

      // Fixing the cylinder's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Checking the order of the cylinder
         if( !superordinate ) {
            error << "   Line " << commandLine_ << ": 'fixed' command is not allowed for subordiante cylinders!\n";
         }

         // Detecting a duplicate fixed command
         if( cylinder.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         cylinder.fixed = true;
      }

      // Making the cylinder invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !cylinder.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         cylinder.visible = false;
      }

      // Setting the POV-Ray texture of the cylinder
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the cylinder
         if( cylinder.textureSet )
            cylinder.texture = povray::LayeredTexture( cylinder.texture, texture );
         else cylinder.texture = texture;

         cylinder.textureSet = true;
      }

      // Ending the 'cylinder' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !centerSet || !radiusSet || !lengthSet ) {
            error << "   Line " << cylinder.line << ": Incomplete 'cylinder' command!\n"
                  << "     Necessary cylinder options are: 'id', 'center', 'radius' and 'length'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'cylinder' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'cylinder' section starting in line " << cylinder.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a plane primitive.
 *
 * \param plane The plane parameters to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractPlane( PlaneParameters& plane, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), normalSet( false ), displacementSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the plane section
   plane.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << plane.line << ": Invalid 'plane' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the plane parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific plane ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the plane ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else plane.id = id;
      }

      // Setting the plane label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the plane label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else plane.label = label;
      }

      // Setting the normal of the plane
      else if( word_.compare( "normal" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate normal command
         if( normalSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'normal' command!\n";
         }
         else normalSet = true;

         // Extracting the normal
         if( !(input_ >> plane.normal) || plane.normal.sqrLength() == real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'normal' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the displacement of the plane
      else if( word_.compare( "displacement" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate displacement command
         if( displacementSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'displacement' command!\n";
         }
         else displacementSet = true;

         // Extracting the displacement
         if( !(input_ >> plane.displacement) ) {
            error << "   Line " << commandLine_ << ": Invalid 'displacement' command!\n";
            errorMode = true;
            input_.clear();
         }
         else plane.displacement *= scaling_[length];
      }

      // Setting the material of the plane
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( plane.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else plane.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( plane.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( plane.material, error );
      }

      // Rotating the plane
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else plane.rotations.push_back( rotation );
      }

      // Translation of the plane
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else plane.translation += translation;
      }

      // Making the plane invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !plane.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         plane.visible = false;
      }

      // Setting the POV-Ray texture of the plane
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the plane
         if( plane.textureSet )
            plane.texture = povray::LayeredTexture( plane.texture, texture );
         else plane.texture = texture;

         plane.textureSet = true;
      }

      // Ending the 'plane' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !normalSet || !displacementSet ) {
            error << "   Line " << plane.line << ": Incomplete 'plane' command!\n"
                  << "     Necessary plane options are: 'id', 'normal' and 'displacement'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'plane' option: '" << word_ << "'!\n";
         errorMode  = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'plane' section starting in line " << plane.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of an agglomerate.
 *
 * \param agglomerate The agglomerate parameters to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractAgglomerate( AgglomerateParameters& agglomerate, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool idSet( false ), labelSet( false ), centerSet( false ), radiusSet( false ), numberSet( false ),
        thresholdSet( false ), linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the agglomerate section
   agglomerate.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << agglomerate.line << ": Invalid 'agglomerate' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the agglomerate parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific agglomerate ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( idSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else idSet = true;

         // Extracting the agglomerate ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.id = id;
      }

      // Setting the agglomerate label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the agglomerate label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.label = label;
      }

      // Setting the position of the agglomerate
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else centerSet = true;

         // Extracting the position
         if( !(input_ >> agglomerate.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.center = agglomerate.center*scaling_[length] + offset_;
      }

      // Setting the number of contained spheres
      else if( word_.compare( "number" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate number command
         if( numberSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'number' command!\n";
         }
         else numberSet = true;

         // Extracting the number of spheres
         UnsignedValue<size_t> number;

         if( !(input_ >> number) || number == 0 ) {
            error << "   Line " << commandLine_ << ": Invalid 'number' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.number = number;
      }

      // Setting the radius of the agglomerate
      else if( word_.compare( "radius" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate radius command
         if( radiusSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'radius' command!\n";
         }
         else radiusSet = true;

         // Extracting the radius
         if( !(input_ >> agglomerate.radius) || agglomerate.radius <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'radius' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.radius *= scaling_[length];
      }

      // Setting the threshold of the agglomerate
      else if( word_.compare( "threshold" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate threshold command
         if( thresholdSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'threshold' command!\n";
         }
         else thresholdSet = true;

         // Extracting the threshold value
         if( !(input_ >> agglomerate.threshold) || agglomerate.threshold < real(0) || agglomerate.threshold > real(1) ) {
            error << "   Line " << commandLine_ << ": Invalid 'threshold' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the material of the agglomerate
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( agglomerate.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else agglomerate.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( agglomerate.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( agglomerate.material, error );
      }

      // Setting the linear velocity of the agglomerate
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> agglomerate.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the agglomerate
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> agglomerate.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.angular *= scaling_[velocity];
      }

      // Translation of the agglomerate
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.translation += translation;
      }

      // Rotating the agglomerate
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else agglomerate.rotations.push_back( rotation );
      }

      // Fixing the agglomerate's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate fixed command
         if( agglomerate.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         agglomerate.fixed = true;
      }

      // Making the agglomerate invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !agglomerate.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         agglomerate.visible = false;
      }

      // Setting the POV-Ray texture of the agglomerate
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the agglomerate
         if( agglomerate.textureSet )
            agglomerate.texture = povray::LayeredTexture( agglomerate.texture, texture );
         else agglomerate.texture = texture;

         agglomerate.textureSet = true;
      }

      // Ending the 'agglomerate' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !idSet || !centerSet || !radiusSet || !thresholdSet ) {
            error << "   Line " << agglomerate.line << ": Incomplete 'agglomerate' command!\n"
                  << "     Necessary agglomerate options are: 'id', 'center', 'radius' and 'threshold'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'agglomerate' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'agglomerate' section starting in line " << agglomerate.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a union.
 *
 * \param u The union parameters to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractUnion( UnionParameters& u, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool cidSet( false ), labelSet( false ), bodySet( false ), linearSet( false ), angularSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the union section
   u.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << u.line << ": Invalid 'union' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the union parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the user-specific union ID
      if( word_.compare( "id" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( cidSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
         }
         else cidSet = true;

         // Extracting the union ID
         UnsignedValue<size_t> id;

         if( !(input_ >> id) ) {
            error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.id = id;
      }

      // Setting the union label
      else if( word_.compare( "label" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate id command
         if( labelSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'label' command!\n";
         }
         else labelSet = true;

         // Extracting the union label
         std::string label;

         if( !(input_ >> label) ) {
            error << "   Line " << commandLine_ << ": Invalid 'label' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.label = label;
      }

      // Setting the position of the union
      else if( word_.compare( "center" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate center command
         if( u.centerSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'center' command!\n";
         }
         else u.centerSet = true;

         // Extracting the position
         if( !(input_ >> u.center) ) {
            error << "   Line " << commandLine_ << ": Invalid 'center' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.center = u.center*scaling_[length] + offset_;
      }

      // Setting the material of the union
      else if( word_.compare( "material" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate material command
         if( u.materialSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'material' command!\n";
         }
         else u.materialSet = true;

         // Extracting a material qualifier
         if( input_ >> std::ws && input_.peek() != '{' )
         {
            if( !(input_ >> word_) ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command!\n";
               input_.clear();
            }
            else if( ( u.material = Material::find( word_ ) ) == invalid_material ) {
               error << "   Line " << commandLine_ << ": Invalid 'material' command, unknown material!\n";
            }
         }

         // Extracting the material parameters
         else extractMaterial( u.material, error );
      }

      // Setting the linear velocity of the union
      else if( word_.compare( "linear" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate linear command
         if( linearSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'linear' command!\n";
         }
         else linearSet = true;

         // Extracting the linear velocity
         if( !(input_ >> u.linear) ) {
            error << "   Line " << commandLine_ << ": Invalid 'linear' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.linear *= scaling_[velocity];
      }

      // Setting the angular velocity of the union
      else if( word_.compare( "angular" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate angular command
         if( angularSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'angular' command!\n";
         }
         else angularSet = true;

         // Extracting the angular velocity
         if( !(input_ >> u.angular) ) {
            error << "   Line " << commandLine_ << ": Invalid 'angular' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.angular *= scaling_[velocity];
      }

      // Translation of the union
      else if( word_.compare( "translate" ) == 0 )
      {
         errorMode = false;

         Vec3 translation;

         if( !(input_ >> translation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.translation += translation;
      }

      // Rotating the union
      else if( word_.compare( "rotate" ) == 0 )
      {
         errorMode = false;

         Vec3 rotation;

         if( !(input_ >> rotation) ) {
            error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
            errorMode = true;
            input_.clear();
         }
         else u.rotations.push_back( rotation );
      }

      // Fixing the union's center of mass
      else if( word_.compare( "fixed" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate fixed command
         if( u.fixed ) {
            error << "   Line " << commandLine_ << ": Duplicate 'fixed' command!\n";
         }

         u.fixed = true;
      }

      // Making the union invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !u.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         u.visible = false;
      }

      // Adding a sphere to the union
      else if( word_.compare( "sphere" ) == 0 )
      {
         errorMode = false;
         bodySet   = true;

         // Extracting the sphere parameters
         Error suberror;
         SphereParameters param;
         if( !extractSphere( param, suberror, true ) ) {
            error << " Subordinate sphere (line " << param.line << ") could not be created:\n";
            error.append( suberror );
         }
         else {
            // Registering the sphere parameters
            u.spheres.push_back( param );
         }
      }

      // Adding a box to the union
      else if( word_.compare( "box" ) == 0 )
      {
         errorMode = false;
         bodySet   = true;

         // Extracting the box parameters
         Error suberror;
         BoxParameters param;
         if( !extractBox( param, suberror, true ) ) {
            error << " Subordinate box (line " << param.line << ") could not be created:\n";
            error.append( suberror );
         }
         else {
            // Registering the box parameters
            u.boxes.push_back( param );
         }
      }

      // Adding a capsule to the union
      else if( word_.compare( "capsule" ) == 0 )
      {
         errorMode = false;
         bodySet   = true;

         // Extracting the capsule parameters
         Error suberror;
         CapsuleParameters param;
         if( !extractCapsule( param, suberror, true ) ) {
            error << " Subordinate capsule (line " << param.line << ") could not be created:\n";
            error.append( suberror );
         }
         else {
            // Registering the capsule parameters
            u.capsules.push_back( param );
         }
      }

      // Adding a cylinder to the union
      else if( word_.compare( "cylinder" ) == 0 )
      {
         errorMode = false;
         bodySet   = true;

         // Extracting the cylinder parameters
         Error suberror;
         CylinderParameters param;
         if( !extractCylinder( param, suberror, true ) ) {
            error << " Subordinate cylinder (line " << param.line << ") could not be created:\n";
            error.append( suberror );
         }
         else {
            // Registering the cylinder parameters
            u.cylinders.push_back( param );
         }
      }

      // Adding a plane to the union
      else if( word_.compare( "plane" ) == 0 )
      {
         errorMode = false;
         bodySet   = true;

         // Extracting the plane parameters
         Error suberror;
         PlaneParameters param;
         if( !extractPlane( param, suberror ) ) {
            error << " Subordinate plane (line " << param.line << ") could not be created:\n";
            error.append( suberror );
         }
         else {
            // Registering the plane parameters
            u.planes.push_back( param );
         }
      }

      // Adding a link to the union
      else if( word_.compare( "link" ) == 0 )
      {
         // Extracting the leading bracket
         // If no leading bracket is found, it is assumed that the bracket was forgotten.
         if( !(input_ >> leadingbracket) ) {
            error << "   Line " << commandLine_ << ": Invalid 'link' command, '{' expected!\n";
            input_.clear();
         }

         bool idSet( false ), body1Set( false ), body2Set( false );
         const size_t linkLine( getLineNumber() );
         LinkParameters link;

         // Extracting the link parameters
         while( input_ >> word_ )
         {
            // Calculating the line number
            commandLine_ = getLineNumber();

            // Setting the user-specific link ID
            if( word_.compare( "id" ) == 0 )
            {
               errorMode = false;

               // Detecting a duplicate id command
               if( idSet ) {
                  error << "   Line " << commandLine_ << ": Duplicate 'id' command!\n";
               }
               else idSet = true;

               // Extracting the link ID
               UnsignedValue<size_t> id( 0 );

               if( !(input_ >> id) ) {
                  error << "   Line " << commandLine_ << ": Invalid 'id' command!\n";
                  errorMode  = true;
                  input_.clear();
               }
               else link.id = id;
            }

            // Setting the first attached body of the link
            else if( word_.compare( "body1" ) == 0 )
            {
               std::string bodyref;

               if( !(input_ >> bodyref) ) {
                  error << "   Line " << commandLine_ << ": Invalid 'body1' command!\n";
                  errorMode = true;
                  input_.clear();
                  continue;
               }

               if( !body1Set ) {
                  body1Set = true;
                  link.body1 = bodyref;
               }
               else {
                  error << "   Line " << commandLine_ << ": Invalid 'body1' command! Body1 has already been specified!\n";
               }
            }

            // Setting the second attached body of the link
            else if( word_.compare( "body2" ) == 0 )
            {
               std::string bodyref;

               if( !(input_ >> bodyref) ) {
                  error << "   Line " << commandLine_ << ": Invalid 'body2' command!\n";
                  errorMode = true;
                  input_.clear();
                  continue;
               }

               if( !body2Set ) {
                  body2Set = true;
                  link.body2 = bodyref;
               }
               else {
                  error << "   Line " << commandLine_ << ": Invalid 'body2' command! Body2 has already been specified!\n";
               }
            }

            // Ending the 'link' command
            else if( word_.compare( "}" ) == 0 )
            {
               errorMode = false;

               // Checking if all necessary options were specified
               if( !idSet || !body1Set || !body2Set ) {
                  error << "   Line " << linkLine << ": Incomplete 'link' command!\n"
                         << "     Necessary link options are: 'id' and two primitives!\n";
               }

               // Assuring the specified bodies are different
               if( link.body1 == link.body2 ) {
                  error << "   Line " << linkLine << ": Invalid 'link' command! Identical primitives specified!\n";
               }

               break;
            }

            // Treatment of unknown options in error mode
            else if( errorMode ) continue;

            // Treatment of unknown options
            else
            {
               error << "   Line " << commandLine_ << ": Unknown 'link' option: '" << word_ << "'!\n";
               errorMode = true;
            }
         }

         // Checking for a duplicate link definition
         for( LinkParameterVector::const_iterator l=u.links.begin(); l!=u.links.end(); ++l ) {
            if( link == *l ) {
               error << "   Line " << linkLine << ": Duplicate link definition!\n";
            }
         }

         // Registering the link
         u.links.push_back( link );
      }

      // Setting the POV-Ray texture of the union
      else if( word_.compare( "texture" ) == 0 )
      {
         errorMode = false;

         // Extracting the new texture
         povray::Texture texture;
         extractTexture( texture, error );

         // Applying the new texture to the union
         if( u.textureSet )
            u.texture = povray::LayeredTexture( u.texture, texture );
         else u.texture = texture;

         u.textureSet = true;
      }

      // Ending the 'union' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !cidSet || !bodySet ) {
            error << "   Line " << u.line << ": Incomplete 'union' command!\n"
                  << "     At least the union ID and one rigid body have to be specified!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'union' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'union' section starting in line " << u.line << "!\n";
   }

   // Checking, whether the 'center' command was combined with a 'plane' command
   if( !u.planes.empty() && ( u.centerSet || linearSet || angularSet ) ) {
      error << "   The 'plane' command cannot be used with the 'center', 'linear' or 'angular' command!\n";
   }

   // Setting the visibility and material of the contained rigid bodies
   if( !error )
   {
      // Configuring all contained spheres
      {
         SphereParameterVector::iterator begin( u.spheres.begin() );
         SphereParameterVector::iterator end  ( u.spheres.end()   );

         for( ; begin!=end; ++begin )
         {
            if( !begin->materialSet && !u.materialSet ) {
               error << "   Line " << begin->line << ": No material information available for sphere!\n";
            }
            else if( !begin->materialSet ) {
               begin->material = u.material;
            }

            if( !u.visible ) begin->visible = false;
         }
      }

      // Configuring all contained boxes
      {
         BoxParameterVector::iterator begin( u.boxes.begin() );
         BoxParameterVector::iterator end  ( u.boxes.end()   );

         for( ; begin!=end; ++begin )
         {
            if( !begin->materialSet && !u.materialSet ) {
               error << "   Line " << begin->line << ": No material information available for box!\n";
            }
            else if( !begin->materialSet ) {
               begin->material = u.material;
            }

            if( !u.visible ) begin->visible = false;
         }
      }

      // Configuring all contained capsules
      {
         CapsuleParameterVector::iterator begin( u.capsules.begin() );
         CapsuleParameterVector::iterator end  ( u.capsules.end()   );

         for( ; begin!=end; ++begin )
         {
            if( !begin->materialSet && !u.materialSet ) {
               error << "   Line " << begin->line << ": No material information available for capsule!\n";
            }
            else if( !begin->materialSet ) {
               begin->material = u.material;
            }

            if( !u.visible ) begin->visible = false;
         }
      }

      // Configuring all contained cylinders
      {
         CylinderParameterVector::iterator begin( u.cylinders.begin() );
         CylinderParameterVector::iterator end  ( u.cylinders.end()   );

         for( ; begin!=end; ++begin )
         {
            if( !begin->materialSet && !u.materialSet ) {
               error << "   Line " << begin->line << ": No material information available for cylinder!\n";
            }
            else if( !begin->materialSet ) {
               begin->material = u.material;
            }

            if( !u.visible ) begin->visible = false;
         }
      }

      // Configuring all contained planes
      {
         PlaneParameterVector::iterator begin( u.planes.begin() );
         PlaneParameterVector::iterator end  ( u.planes.end()   );

         for( ; begin!=end; ++begin )
         {
            if( !begin->materialSet && !u.materialSet ) {
               error << "   Line " << begin->line << ": No material information available for plane!\n";
            }
            else if( !begin->materialSet ) {
               begin->material = u.material;
            }

            if( !u.visible ) begin->visible = false;
         }
      }
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a spring.
 *
 * \param spring The spring parameters to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractSpring( SpringParameters& spring, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool body1Set( false ), body2Set( false ), anchor1Set( false ), anchor2Set( false ),
        stiffnessSet( false ), dampingSet( false ), lengthSet( false );
   LeadingBracket leadingbracket;

   // Storing the line number of the spring section
   spring.line = getLineNumber();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << spring.line << ": Invalid 'spring' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the spring parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the first attached body of the spring
      if( word_.compare( "body1" ) == 0 )
      {
         std::string bodyref;

         if( !(input_ >> bodyref) ) {
            error << "   Line " << commandLine_ << ": Invalid 'body1' command!\n";
            errorMode = true;
            input_.clear();
            continue;
         }

         if( !body1Set ) {
            body1Set = true;
            spring.body1 = bodyref;
         }
         else {
            error << "   Line " << commandLine_ << ": Invalid 'body1' command! Body1 has already been specified!\n";
         }
      }

      // Setting the second attached body of the spring
      else if( word_.compare( "body2" ) == 0 )
      {
         std::string bodyref;

         if( !(input_ >> bodyref) ) {
            error << "   Line " << commandLine_ << ": Invalid 'body2' command!\n";
            errorMode = true;
            input_.clear();
            continue;
         }

         if( !body2Set ) {
            body2Set = true;
            spring.body2 = bodyref;
         }
         else {
            error << "   Line " << commandLine_ << ": Invalid 'body2' command! Body2 has already been specified!\n";
         }
      }

      // Setting the anchor point of the first attached rigid body
      else if( word_.compare( "anchor1" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate anchor1 command
         if( anchor1Set ) {
            error << "   Line " << commandLine_ << ": Duplicate 'anchor1' command!\n";
         }
         else anchor1Set = true;

         // Extracting the second anchor point
         if( !(input_ >> spring.anchor1) ) {
            error << "   Line " << commandLine_ << ": Invalid 'anchor1' command!\n";
            errorMode = true;
            input_.clear();
         }
         else spring.anchor1 *= scaling_[length];
      }

      // Setting the anchor point of the second attached rigid body
      else if( word_.compare( "anchor2" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate anchor2 command
         if( anchor2Set ) {
            error << "   Line " << commandLine_ << ": Duplicate 'anchor2' command!\n";
         }
         else anchor2Set = true;

         // Extracting the second anchor point
         if( !(input_ >> spring.anchor2) ) {
            error << "   Line " << commandLine_ << ": Invalid 'anchor2' command!\n";
            errorMode = true;
            input_.clear();
         }
         else spring.anchor2 *= scaling_[length];
      }

      // Setting the stiffness of the spring
      else if( word_.compare( "stiffness" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate stiffness command
         if( stiffnessSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'stiffness' command!\n";
         }
         else stiffnessSet = true;

         // Extracting the stiffness
         if( !(input_ >> spring.stiffness) || spring.stiffness <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'stiffness' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the damping of the spring
      else if( word_.compare( "damping" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate damping command
         if( dampingSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'damping' command!\n";
         }
         else dampingSet = true;

         // Extracting the damping factor
         if( !(input_ >> spring.damping) || spring.damping <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'damping' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the rest length of the spring
      else if( word_.compare( "length" ) == 0 )
      {
         errorMode = true;

         // Detecting a duplicate length command
         if( lengthSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'length' command!\n";
         }
         else lengthSet = true;

         // Extracting the rest length
         if( !(input_ >> spring.length) || spring.length <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'length' command!\n";
            errorMode = true;
            input_.clear();
         }
         else spring.length *= scaling_[length];
      }

      // Making the spring invisible
      else if( word_.compare( "invisible" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate invisible command
         if( !spring.visible ) {
            error << "   Line " << commandLine_ << ": Duplicate 'invisible' command!\n";
         }

         spring.visible = false;
      }

      // Ending the 'spring' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !body1Set || !body2Set || !stiffnessSet || !dampingSet ) {
            error << "   Line " << spring.line << ": Incomplete 'spring' command!\n"
                  << "     Necessary spring options are: two bodies, 'stiffness' and 'damping'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'spring' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'spring' section starting in line " << spring.line << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a material.
 *
 * \param material The material to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractMaterial( MaterialID& material, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool nameSet( false ), densitySet( false ), corSet( false ),
        csfSet( false ), cdfSet( false ), poissonSet( false ), youngSet( false ),
        stiffnessSet( false ), dampingNSet( false ), dampingTSet( false );
   const size_t materialLine( getLineNumber() );
   std::string name;
   real density( 0 ), cor( 0 ), csf( 0 ), cdf( 0 ), poisson( 0 ), young( 0 ),
        stiffness( 0 ), dampingN( 0 ), dampingT( 0 );
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << materialLine << ": Invalid 'material' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the material parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Extracting the material name
      if( word_.compare( "name" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate name command
         if( nameSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'name' command!\n";
         }
         else nameSet = true;

         if( !(input_ >> name) ) {
            error << "   Line " << commandLine_ << ": Invalid 'name' command!\n";
            errorMode = true;
            input_.clear();
         }
         else if( Material::find( name ) != invalid_material ) {
            error << "   Line " << commandLine_ << ": Material identifier already used!\n";
         }
      }

      // Setting the material density
      else if( word_.compare( "density" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate density command
         if( densitySet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'density' command!\n";
         }
         else densitySet = true;

         if( !(input_ >> density) || density <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'density' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the coefficient of restitution of the material
      else if( word_.compare( "restitution" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate restitution command
         if( corSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'restitution' command!\n";
         }
         else corSet = true;

         if( !(input_ >> cor) || cor < real(0) || cor > real(1) ) {
            error << "   Line " << commandLine_ << ": Invalid 'restitution' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the coefficient of static friction of the material
      else if( word_.compare( "static" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate static command
         if( csfSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'static' command!\n";
         }
         else csfSet = true;

         if( !(input_ >> csf) || csf < real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'static' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting the coefficient of dynamic friction of the material
      else if( word_.compare( "dynamic" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate dynamic command
         if( cdfSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'dynamic' command!\n";
         }
         else cdfSet = true;

         if( !(input_ >> cdf) || cdf < real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'dynamic' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting Poisson's ratio of the material
      else if( word_.compare( "poisson" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate poisson command
         if( poissonSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'poisson' command!\n";
         }
         else poissonSet = true;

         if( !(input_ >> poisson) || poisson < real(-1) || poisson > real(0.5) ) {
            error << "   Line " << commandLine_ << ": Invalid 'poisson' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting Young's modulus of the material
      else if( word_.compare( "young" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate young command
         if( youngSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'young' command!\n";
         }
         else youngSet = true;

         if( !(input_ >> young) || young < real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'young' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting stiffness in normal direction of the material
      else if( word_.compare( "stiffness" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate young command
         if( stiffnessSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'stiffness' command!\n";
         }
         else stiffnessSet = true;

         if( !(input_ >> stiffness) || stiffness <= real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'stiffness' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting damping coefficient in normal direction of the material
      else if( word_.compare( "dampingn" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate young command
         if( dampingNSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'dampingn' command!\n";
         }
         else dampingNSet = true;

         if( !(input_ >> dampingN) || dampingN < real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'dampingn' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Setting damping coefficient in tangential direction of the material
      else if( word_.compare( "dampingt" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate young command
         if( dampingTSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'dampingt' command!\n";
         }
         else dampingTSet = true;

         if( !(input_ >> dampingT) || dampingT < real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'dampingt' command!\n";
            errorMode = true;
            input_.clear();
         }
      }

      // Ending the 'material' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !nameSet || !densitySet || !corSet || !csfSet || !cdfSet || !poissonSet || !youngSet || !stiffnessSet || !dampingNSet || !dampingTSet ) {
            error << "   Line " << materialLine << ": Incomplete 'material' command!\n"
                  << "     Necessary material options are 'name', 'density', 'restitution', 'static', 'dynamic', 'poisson', 'young', 'stiffness', 'dampingn' and 'dampingt'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'material' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'material' section starting in line " << materialLine << "!\n";
   }

   // Creating the new material
   if( !error ) {
      material = createMaterial( name, density, cor, csf, cdf, poisson, young, stiffness, dampingN, dampingT );
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray texture.
 *
 * \param texture The POV-Ray texture to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractTexture( povray::Texture& texture, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   const size_t textureLine( getLineNumber() );
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << textureLine << ": Invalid 'texture' command, '{' expected!\n";
      input_.clear();
   }

   // Resetting the texture
   texture.reset();

   // Extracting the first keyword
   commandLine_ = getLineNumber();
   if( !(input_ >> word_) ) {
      error << "   Line " << textureLine << ": Invalid 'texture' command!\n";
      input_.clear();
      return false;
   }

   // Checking for an empty texture
   if( word_.compare( "}" ) == 0 )
      return !error;

   // Extracting a declared texture
   if( word_.compare( "use" ) == 0 )
   {
      // Extracting the texture identifier
      if( !(input_ >> word_) || word_.compare( "}" ) == 0 ) {
         error << "   Line " << commandLine_ << ": Missing 'texture' identifier after 'use' command!\n";
         input_.clear();
         return false;
      }

      // Creating a new custom texture
      povray::CustomTexture custom( word_ );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a texture scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Scale( scale ) );
         }

         // Extracting a texture translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Translation( translation ) );
         }

         // Extracting a texture rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Rotation( rotation ) );
         }

         // Ending the custom texture
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'custom texture' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      texture = custom;
   }

   // Extracting a tiled texture
   else if( word_.compare( "tiles" ) == 0 )
   {
      povray::Texture texture1, texture2;

      // Extracting the leading bracket
      // If no leading bracket is found, it is assumed that the bracket was forgotten.
      if( !(input_ >> leadingbracket) ) {
         error << "   Line " << commandLine_ << ": Invalid 'tiles' command, '{' expected!\n";
         input_.clear();
      }

      // Extracting the first texture
      if( !(input_ >> word_) || word_.compare( "texture" ) != 0 ) {
         error << "   Line " << commandLine_ << ": Invalid 'tiles' command! Expected 'texture' command!\n";
         errorMode = true;
         input_.clear();
         finishBlock();
      }
      else extractTexture( texture1, error );

      // Extracting the second texture
      if( !(input_ >> word_) || word_.compare( "texture" ) != 0 ) {
         error << "   Line " << commandLine_ << ": Invalid 'tiles' command! No second 'texture' command found!\n";
         errorMode = true;
         input_.clear();
         finishBlock();
      }
      else extractTexture( texture2, error );

      // Extracting the trailing bracket
      TrailingBracket trailingbracket;
      if( !(input_ >> trailingbracket) ) {
         error << "   '}' expected after 'tiles' section starting in line " << commandLine_ << "!\n";
         errorMode = true;
         input_.clear();
      }

      povray::TiledTexture tiledTexture( texture1, texture2 );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a texture scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else tiledTexture.add( povray::Scale( scale ) );
         }

         // Extracting a texture translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else tiledTexture.add( povray::Translation( translation ) );
         }

         // Extracting a texture rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else tiledTexture.add( povray::Rotation( rotation ) );
         }

         // Ending the 'texture' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'texture' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      texture = tiledTexture;
   }

   // Extracting a plain texture
   else
   {
      bool pigmentSet( false ), finishSet( false ), normalSet( false );
      povray::PlainTexture plainTexture;

      do {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the texture pigment
         if( word_.compare( "pigment" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate pigment command
            if( pigmentSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'pigment' command!\n";
            }
            else pigmentSet = true;

            // Extracting the pigment
            povray::Pigment pigment;
            if( extractPigment( pigment, error ) )
               plainTexture.add( pigment );
         }

         // Extracting the texture finish
         else if( word_.compare( "finish" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate finish command
            if( finishSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'finish' command!\n";
            }
            else finishSet = true;

            // Extracting the finish
            povray::Finish finish;
            if( extractFinish( finish, error ) )
               plainTexture.add( finish );
         }

         // Extracting the texture normal
         else if( word_.compare( "normal" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate normal command
            if( normalSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'normal' command!\n";
            }
            else normalSet = true;

            // Extracting the normal
            povray::Normal normal;
            if( extractNormal( normal, error ) )
               plainTexture.add( normal );
         }

         // Extracting a texture scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else plainTexture.add( povray::Scale( scale ) );
         }

         // Extracting a texture translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else plainTexture.add( povray::Translation( translation ) );
         }

         // Extracting a texture rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else plainTexture.add( povray::Rotation( rotation ) );
         }

         // Ending the 'texture' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'texture' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }
      while( input_ >> word_ );

      texture = plainTexture;
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'texture' section starting in line " << textureLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray pigment.
 *
 * \param pigment The POV-Ray pigment to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractPigment( povray::Pigment& pigment, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   const size_t pigmentLine( getLineNumber() );
   LeadingBracket leadingbracket;
   TrailingBracket trailingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << pigmentLine << ": Invalid 'pigment' command, '{' and pigment type expected!\n";
      input_.clear();
   }

   // Resetting the pigment
   pigment.reset();

   // Extracting the first keyword
   if( !(input_ >> word_) ) {
      error << "   Line " << pigmentLine << ": Invalid 'pigment' command, pigment type expected!\n";
      input_.clear();
      return false;
   }

   // Checking for an empty pigment
   if( word_.compare( "}" ) == 0 )
      return !error;

   // Checking the type of the pigment
   if( word_.compare( "use"       ) != 0 &&
       word_.compare( "color"     ) != 0 &&
       word_.compare( "agate"     ) != 0 &&
       word_.compare( "bozo"      ) != 0 &&
       word_.compare( "granite"   ) != 0 &&
       word_.compare( "marble"    ) != 0 &&
       word_.compare( "spotted"   ) != 0 &&
       word_.compare( "radial"    ) != 0 &&
       word_.compare( "image_map" ) != 0 ) {
      error << "   Line " << commandLine_ << ": Invalid 'pigment' command, pigment type expected!\n";
      input_.clear();
      finishBlock();
      return false;
   }

   // Extracting a custom pigment
   if( word_.compare( "use" ) == 0 )
   {
      // Extracting the pigment identifier
      if( !(input_ >> word_) || word_.compare( "}" ) == 0 ) {
         error << "   Line " << commandLine_ << ": Missing 'pigment' identifier after 'use' command!\n";
         input_.clear();
         return false;
      }

      // Creating a new custom pigment
      povray::CustomPigment custom( word_ );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Rotation( rotation ) );
         }

         // Ending the custom pigment
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'custom pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = custom;
   }

   // Extracting a single color pigment
   else if( word_.compare( "color" ) == 0 )
   {
      povray::Color color;

      // Extracting the pigment color
      if( !(input_ >> color) ) {
         error << "   Line " << commandLine_ << ": Invalid 'color' command!\n";
         errorMode = true;
         input_.clear();
      }

      povray::ColorPigment colorPigment( color );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else colorPigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else colorPigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else colorPigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'color pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = colorPigment;
   }

   // Extracting an agate pigment
   else if( word_.compare( "agate" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in agate pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in agate pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::AgatePigment agatePigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agatePigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'agate pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = agatePigment;
   }

   // Extracting a bozo pigment
   else if( word_.compare( "bozo" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in bozo pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in bozo pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::BozoPigment bozoPigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoPigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'bozo pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = bozoPigment;
   }

   // Extracting a granite pigment
   else if( word_.compare( "granite" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in granite pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in granite pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::GranitePigment granitePigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else granitePigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'granite pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = granitePigment;
   }

   // Extracting a marble pigment
   else if( word_.compare( "marble" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in marble pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in marble pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::MarblePigment marblePigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marblePigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'marble pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = marblePigment;
   }

   // Extracting a spotted pigment
   else if( word_.compare( "spotted" ) == 0 )
   {
      bool frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in spotted pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in spotted pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::SpottedPigment spottedPigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the frequency modifier
         if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedPigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedPigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedPigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedPigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedPigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'spotted pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = spottedPigment;
   }

   // Extracting a radial pigment
   else if( word_.compare( "radial" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      povray::ColorMap colormap;

      // Extracting the color map
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "color_map" ) != 0 )
      {
         error << "   Line " << commandLine_ << ": Missing 'color_map' command in radial pigment!\n";
         errorMode = true;
         input_.clear();
      }
      else if( !(input_ >> leadingbracket >> colormap >> trailingbracket) )
      {
         error << "   Line " << commandLine_ << ": Invalid 'color_map' command in radial pigment!\n";
         errorMode = true;
         input_.clear();
         skipBlock();
      }

      povray::RadialPigment radialPigment( colormap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else radialPigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'radial pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = radialPigment;
   }

   // Extracting an image map pigment
   else if( word_.compare( "image_map" ) == 0 )
   {
      povray::ImageMap imagemap( povray::gif, "invalid.gif", povray::planar );  // Temporal image map
      extractImageMap( imagemap, error );

      povray::ImagePigment imagePigment( imagemap );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else imagePigment.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else imagePigment.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else imagePigment.add( povray::Rotation( rotation ) );
         }

         // Ending the 'pigment' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'image pigment' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      pigment = imagePigment;
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'pigment' section starting in line " << pigmentLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray finish.
 *
 * \param finish The POV-Ray finish to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractFinish( povray::Finish& finish, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool ambientSet( false ), diffuseSet( false ), phongSet( false ),
        phongSizeSet( false ), specularSet( false ), roughnessSet( false ),
        reflectionSet( false ), refractionSet( false );
   const size_t finishLine( getLineNumber() );
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << finishLine << ": Invalid 'finish' command, '{' expected!\n";
      input_.clear();
   }

   // Resetting the finish
   finish.reset();

   // Extracting the first keyword
   if( !(input_ >> word_) ) {
      error << "   Line " << finishLine << ": Invalid 'finish' command!\n";
      input_.clear();
      return false;
   }

   // Extracting a declared finish
   if( word_.compare( "use" ) == 0 )
   {
      // Extracting the finish identifier
      commandLine_ = getLineNumber();
      if( !(input_ >> word_) || word_.compare( "}" ) == 0 ) {
         error << "   Line " << commandLine_ << ": Missing 'finish' identifier after 'use' command!\n";
         input_.clear();
         return false;
      }

      // Creating a new custom finish
      povray::CustomFinish custom( word_ );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Ending the 'finish' command
         if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'custom finish' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      finish = custom;
   }

   // Extracting a plain finish
   else
   {
      do {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Setting the ambient light
         if( word_.compare( "ambient" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate ambient command
            if( ambientSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'ambient' command!\n";
            }
            else ambientSet = true;

            // Extracting the ambient value
            real ambient(0);

            if( !(input_ >> ambient) || ambient < real(0) || ambient > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'ambient' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Ambient( ambient ) );
         }

         // Setting the diffuse light
         else if( word_.compare( "diffuse" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate diffuse command
            if( diffuseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'diffuse' command!\n";
            }
            else diffuseSet = true;

            // Extracting the diffuse value
            real diffuse(0);

            if( !(input_ >> diffuse) || diffuse < real(0) || diffuse > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'diffuse' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Diffuse( diffuse ) );
         }

         // Setting the phong highlight
         else if( word_.compare( "phong" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phong command
            if( phongSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phong' command!\n";
            }
            else phongSet = true;

            // Extracting the phong value
            real phong(0);

            if( !(input_ >> phong) || phong < real(0) || phong > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phong' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Phong( phong ) );
         }

         // Setting the size of the phong highlight
         else if( word_.compare( "phong_size" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phong size command
            if( phongSizeSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phong_size' command!\n";
            }
            else phongSizeSet = true;

            // Extracting the phong size
            real size(0);

            if( !(input_ >> size) || size < real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phong_size' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::PhongSize( size ) );
         }

         // Setting the specular highlight
         else if( word_.compare( "specular" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate specular command
            if( specularSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'specular' command!\n";
            }
            else specularSet = true;

            // Extracting the specular value
            real specular(0);

            if( !(input_ >> specular) || specular < real(0) || specular > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'specular' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Specular( specular ) );
         }

         // Setting the roughness of the specular highlight
         else if( word_.compare( "roughness" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate roughness command
            if( roughnessSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'roughness' command!\n";
            }
            else roughnessSet = true;

            // Extracting the roughness
            real roughness(0);

            if( !(input_ >> roughness) || roughness <= real(0) || roughness > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'roughness' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Roughness( roughness ) );
         }

         // Setting the reflection
         else if( word_.compare( "reflection" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate reflection command
            if( reflectionSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'reflection' command!\n";
            }
            else reflectionSet = true;

            // Extracting the reflection value
            real reflection(0);

            if( !(input_ >> reflection) || reflection < real(0) || reflection > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'reflection' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Reflection( reflection ) );
         }

         // Setting the refraction
         else if( word_.compare( "refraction" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate refraction command
            if( refractionSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'refraction' command!\n";
            }
            else refractionSet = true;

            // Extracting the refraction value
            real refraction(0);

            if( !(input_ >> refraction) || refraction < real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'refraction' command!\n";
               errorMode = true;
               input_.clear();
            }
            else finish.add( povray::Refraction( refraction ) );
         }

         // Ending the 'finish' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'finish' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }
      while( input_ >> word_ );
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'finish' section starting in line " << finishLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray normal.
 *
 * \param normal The POV-Ray normal to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractNormal( povray::Normal& normal, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   const size_t normalLine( getLineNumber() );
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << normalLine << ": Invalid 'normal' command, '{' expected!\n";
      input_.clear();
   }

   // Resetting the normal
   normal.reset();

   // Extracting the first keyword
   if( !(input_ >> word_) ) {
      error << "   Line " << normalLine << ": Invalid 'normal' command, normal type expected!\n";
      input_.clear();
      return false;
   }

   // Checking for an empty normal
   if( word_.compare( "}" ) == 0 )
      return !error;

   // Checking the type of the normal
   if( word_.compare( "use"      ) != 0 &&
       word_.compare( "agate"    ) != 0 &&
       word_.compare( "bozo"     ) != 0 &&
       word_.compare( "bumps"    ) != 0 &&
       word_.compare( "dents"    ) != 0 &&
       word_.compare( "granite"  ) != 0 &&
       word_.compare( "marble"   ) != 0 &&
       word_.compare( "ripples"  ) != 0 &&
       word_.compare( "spotted"  ) != 0 &&
       word_.compare( "waves"    ) != 0 &&
       word_.compare( "wrinkles" ) != 0 ) {
      error << "   Line " << commandLine_ << ": Unknown 'normal' type: '" << word_ << "'!\n";
      input_.clear();
      finishBlock();
      return false;
   }

   // Extracting a custom normal
   if( word_.compare( "use" ) == 0 )
   {
      // Extracting the normal identifier
      if( !(input_ >> word_) || word_.compare( "}" ) == 0 ) {
         error << "   Line " << commandLine_ << ": Missing 'normal' identifier after 'use' command!\n";
         input_.clear();
         return false;
      }

      // Creating a new custom normal
      povray::CustomNormal custom( word_ );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting a scale transformation
         if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else custom.add( povray::Rotation( rotation ) );
         }

         // Ending the custom normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'custom normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = custom;
   }

   // Extracting an agate normal
   else if( word_.compare( "agate" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the agate depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'agate' depth parameter!\n";
         input_.clear();
      }

      // Creating a new agate normal
      povray::AgateNormal agateNormal( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else agateNormal.add( povray::Rotation( rotation ) );
         }

         // Ending the agate normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'agate normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = agateNormal;
   }

   // Extracting a bozo normal
   else if( word_.compare( "bozo" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the bozo depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'bozo' depth parameter!\n";
         input_.clear();
      }

      // Creating a new bozo normal
      povray::BozoNormal bozoNormal( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bozoNormal.add( povray::Rotation( rotation ) );
         }

         // Ending the bozo normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'bozo normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = bozoNormal;
   }

   // Extracting a bumps normal
   else if( word_.compare( "bumps" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ), octavesSet( false );
      real depth( 0 );

      // Extracting the bumps depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'normal' depth parameter!\n";
         input_.clear();
      }

      // Creating a new bumps normal
      povray::Bumps bumps( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Octaves( octaves ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else bumps.add( povray::Rotation( rotation ) );
         }

         // Ending the bumps normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'bumps normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = bumps;
   }

   // Extracting a dents normal
   else if( word_.compare( "dents" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ), octavesSet( false );
      real depth( 0 );

      // Extracting the dents depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'normal' depth parameter!\n";
         input_.clear();
      }

      // Creating a new dents normal
      povray::Dents dents( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Octaves( octaves ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else dents.add( povray::Rotation( rotation ) );
         }

         // Ending the dents normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'dents normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = dents;
   }

   // Extracting a granite normal
   else if( word_.compare( "granite" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the granite depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'granite' depth parameter!\n";
         input_.clear();
      }

      // Creating a new granite normal
      povray::GraniteNormal graniteNormal( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else graniteNormal.add( povray::Rotation( rotation ) );
         }

         // Ending the granite normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'granite normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = graniteNormal;
   }

   // Extracting a marble normal
   else if( word_.compare( "marble" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the marble depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'marble' depth parameter!\n";
         input_.clear();
      }

      // Creating a new marble normal
      povray::MarbleNormal marbleNormal( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else marbleNormal.add( povray::Rotation( rotation ) );
         }

         // Ending the marble normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'marble normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = marbleNormal;
   }

   // Extracting a ripples normal
   else if( word_.compare( "ripples" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the ripples depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'ripples' depth parameter!\n";
         input_.clear();
      }

      // Creating a new ripples normal
      povray::Ripples ripples( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else ripples.add( povray::Rotation( rotation ) );
         }

         // Ending the ripples normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'ripples normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = ripples;
   }

   // Extracting a spotted normal
   else if( word_.compare( "spotted" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the spotted depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'spotted' depth parameter!\n";
         input_.clear();
      }

      // Creating a new spotted normal
      povray::SpottedNormal spottedNormal( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spottedNormal.add( povray::Rotation( rotation ) );
         }

         // Ending the spotted normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'spotted normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = spottedNormal;
   }

   // Extracting a waves normal
   else if( word_.compare( "waves" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ),
           octavesSet( false ), frequencySet( false ), phaseSet( false );
      real depth( 0 );

      // Extracting the dents depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'waves' depth parameter!\n";
         input_.clear();
      }

      povray::Waves waves( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Octaves( octaves ) );
         }

         // Extracting the frequency modifier
         else if( word_.compare( "frequency" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate frequency command
            if( frequencySet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'frequency' command!\n";
            }
            else frequencySet = true;

            // Extracting the frequency value
            real frequency( 0 );

            if( !(input_ >> frequency) || frequency == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'frequency' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Frequency( frequency ) );
         }

         // Extracting the phase modifier
         else if( word_.compare( "phase" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate phase command
            if( phaseSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'phase' command!\n";
            }
            else phaseSet = true;

            // Extracting the phase value
            real phase( 0 );

            if( !(input_ >> phase) || phase < real(0) || phase > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'phase' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Phase( phase ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else waves.add( povray::Rotation( rotation ) );
         }

         // Ending the waves normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'waves normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = waves;
   }

   // Extracting a wrinkles normal
   else if( word_.compare( "wrinkles" ) == 0 )
   {
      bool turbulenceSet( false ), omegaSet( false ), lambdaSet( false ), octavesSet( false );
      real depth( 0 );

      // Extracting the wrinkles depth
      commandLine_ = getLineNumber();
      if( !(input_ >> depth) || depth < real(0) || depth > real(1) ) {
         error << "   Line " << commandLine_ << ": Invalid 'wrinkles' depth parameter!\n";
         input_.clear();
      }

      povray::Wrinkles wrinkles( depth );

      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the turbulence modifier
         if( word_.compare( "turbulence" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate turbulence command
            if( turbulenceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'turbulence' command!\n";
            }
            else turbulenceSet = true;

            // Extracting the turbulence value
            real turbulence( 0 );

            if( !(input_ >> turbulence) || turbulence < real(0) || turbulence > real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'turbulence' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Turbulence( turbulence ) );
         }

         // Extracting the omega modifier
         else if( word_.compare( "omega" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate omega command
            if( omegaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'omega' command!\n";
            }
            else omegaSet = true;

            // Extracting the omega value
            real omega( 0 );

            if( !(input_ >> omega) || omega <= real(0) || omega >= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'omega' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Omega( omega ) );
         }

         // Extracting the lambda modifier
         else if( word_.compare( "lambda" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate lambda command
            if( lambdaSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'lambda' command!\n";
            }
            else lambdaSet = true;

            // Extracting the lambda value
            real lambda( 0 );

            if( !(input_ >> lambda) || lambda <= real(1) ) {
               error << "   Line " << commandLine_ << ": Invalid 'lambda' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Lambda( lambda ) );
         }

         // Extracting the octaves modifier
         else if( word_.compare( "octaves" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate octaves command
            if( octavesSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'octaves' command!\n";
            }
            else octavesSet = true;

            // Extracting the octaves value
            UnsignedValue<unsigned int> octaves( 0 );

            if( !(input_ >> octaves) || octaves == 0 ) {
               error << "   Line " << commandLine_ << ": Invalid 'octaves' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Octaves( octaves ) );
         }

         // Extracting a scale transformation
         else if( word_.compare( "scale" ) == 0 )
         {
            errorMode = false;

            // Extracting the scale value
            real scale( 0 );

            if( !(input_ >> scale) || scale == real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'scale' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Scale( scale ) );
         }

         // Extracting a translation transformation
         else if( word_.compare( "translate" ) == 0 )
         {
            errorMode = false;

            // Extracting the translation vector
            Vec3 translation;

            if( !(input_ >> translation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'translate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Translation( translation ) );
         }

         // Extracting a rotation transformation
         else if( word_.compare( "rotate" ) == 0 )
         {
            errorMode = false;

            // Extracting the rotation vector
            Vec3 rotation;

            if( !(input_ >> rotation) ) {
               error << "   Line " << commandLine_ << ": Invalid 'rotate' command!\n";
               errorMode = true;
               input_.clear();
            }
            else wrinkles.add( povray::Rotation( rotation ) );
         }

         // Ending the wrinkles normal
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'wrinkles normal' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      normal = wrinkles;
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'normal' section starting in line " << normalLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray image map.
 *
 * \param imagemap The POV-Ray image map to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractImageMap( povray::ImageMap& imagemap, Error& error )
{
   bool endTagFound( false ), errorMode( false ), mapTypeSet( false ), repeat( true );
   povray::ImageType imageType( povray::gif );
   povray::MappingType mappingType( povray::planar );
   InputString filename;
   const size_t imageMapLine( getLineNumber() );
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << imageMapLine << ": Invalid 'image_map' command, '{' expected!\n";
      input_.clear();
   }

   // Extracting the image type
   if( !(input_ >> word_) ) {
      error << "   Line " << imageMapLine << ": Invalid 'image_map' command, image type expected!\n";
      input_.clear();
      finishBlock();
      return false;
   }
   else if( word_.compare( "gif" ) == 0 ) {
      imageType = povray::gif;
   }
   else if( word_.compare( "tga" ) == 0 ) {
      imageType = povray::tga;
   }
   else if( word_.compare( "iff" ) == 0 ) {
      imageType = povray::iff;
   }

   // Extracting the image file name
   if( !(input_ >> filename) ) {
      error << "   Line " << imageMapLine << ": Invalid 'image_map' command, invalid file name!\n";
      input_.clear();
      finishBlock();
      return false;
   }

   // Extracting the optional image map parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Extracting the map type of the image map
      if( word_.compare( "map_type" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate map_type command
         if( mapTypeSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'map_type' command!\n";
         }
         else mapTypeSet = true;

         // Extracting the map type
         if( !(input_ >> word_) ) {
            error << "   Line " << commandLine_ << ": Invalid 'map_type' command!\n";
            errorMode = true;
            input_.clear();
         }
         else if( word_.compare( "planar" ) == 0 ) {
            mappingType = povray::planar;
         }
         else if( word_.compare( "spherical" ) == 0 ) {
            mappingType = povray::spherical;
         }
         else {
            error << "   Line " << commandLine_ << ": Unknown mapping type!\n";
         }
      }

      // Extracting the once keyword
      else if( word_.compare( "once" ) == 0 )
      {
         errorMode = false;

         if( !repeat ) {
            error << "   Line " << commandLine_ << ": Duplicate 'once' command!\n";
         }
         else repeat = false;
      }

      // Ending the 'image_map' command
      else if( word_.compare( "}" ) == 0 )
      {
         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'image_map' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'image_map' section starting in line " << imageMapLine << "!\n";
   }

   // Creating the new image map
   imagemap = povray::ImageMap( imageType, filename.str(), mappingType, repeat );

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray camera.
 *
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractCamera( Error& error )
{
   bool endTagFound( false ), errorMode( false );
   bool locationSet( false ), lookAtSet( false ), skySet( false );
   const size_t cameraLine( getLineNumber() );
   LeadingBracket leadingbracket;

   // Getting a handle to the POV-Ray camera
   povray::CameraID camera = povray::theCamera();

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << cameraLine << ": Invalid 'camera' command, '{' expected!\n";
      input_.clear();
   }

   // Resetting the camera
   camera->reset();

   // Extracting the texture parameters
   while( input_ >> word_ )
   {
      // Calculating the line number
      commandLine_ = getLineNumber();

      // Setting the location of the camera
      if( word_.compare( "location" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate location command
         if( locationSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'location' command!\n";
         }
         else locationSet = true;

         // Extracting the camera location
         Vec3 location;

         if( !(input_ >> location) ) {
            error << "   Line " << commandLine_ << ": Invalid 'location' command!\n";
            errorMode = true;
            input_.clear();
         }
         else camera->setLocation( location );
      }

      // Setting the focus point of the camera
      else if( word_.compare( "look_at" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate look at command
         if( lookAtSet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'look_at' command!\n";
         }
         else lookAtSet = true;

         // Extracting the camera focus point
         Vec3 focus;

         if( !(input_ >> focus) ) {
            error << "   Line " << commandLine_ << ": Invalid 'look_at' command!\n";
            errorMode = true;
            input_.clear();
         }
         else camera->setFocus( focus );
      }

      // Setting the up/sky-direction of the camera
      else if( word_.compare( "sky" ) == 0 )
      {
         errorMode = false;

         // Detecting a duplicate sky command
         if( skySet ) {
            error << "   Line " << commandLine_ << ": Duplicate 'sky' command!\n";
         }
         else skySet = true;

         // Extracting the camera up/sky-direction
         Vec3 sky;

         if( !(input_ >> sky) || sky.sqrLength() == real(0) ) {
            error << "   Line " << commandLine_ << ": Invalid 'sky' command!\n";
            errorMode = true;
            input_.clear();
         }
         else camera->setSky( sky );
      }

      // Ending the 'camera' command
      else if( word_.compare( "}" ) == 0 )
      {
         // Checking if all necessary options were specified
         if( !locationSet || !lookAtSet ) {
            error << "   Line " << cameraLine << ": Incomplete 'camera' command!\n"
                     << "     Necessary camera options are: 'location' and 'look_at'!\n";
         }

         endTagFound = true;
         break;
      }

      // Treatment of unknown options in error mode
      else if( errorMode ) continue;

      // Treatment of unknown options
      else
      {
         error << "   Line " << commandLine_ << ": Unknown 'camera' option: '" << word_ << "'!\n";
         errorMode = true;
      }
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'camera' section starting in line " << cameraLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extraction of a POV-Ray light source.
 *
 * \param lightsource The POV-Ray light source to be extracted.
 * \param error The error container for possible error messages.
 * \return \a true if no error was detected, \a false if not.
 */
bool BodyReader::extractLightSource( povray::LightSource& lightsource, Error& error )
{
   bool endTagFound( false ), errorMode( false );
   const size_t lightsourceLine( getLineNumber() );
   Vec3 gpos;
   povray::Color color;
   LeadingBracket leadingbracket;

   // Extracting the leading bracket
   // If no leading bracket is found, it is assumed that the bracket was forgotten.
   if( !(input_ >> leadingbracket) ) {
      error << "   Line " << lightsourceLine << ": Invalid 'light_source' command, '{' expected!\n";
      input_.clear();
   }

   // Resetting the normal
   lightsource.reset();

   // Extracting the location and color of the light source
   if( !(input_ >> gpos >> color >> word_) ) {
      error << "   Line " << lightsourceLine << ": Invalid 'light_source' command!\n";
      input_.clear();
      finishBlock();
      return false;
   }

   // Checking for a final semicolon
   if( word_.compare( "}" ) == 0 ) {
      lightsource = povray::PointLight( gpos, color );
      return !error;
   }

   // Extracting a spotlight
   else if( word_.compare( "spotlight" ) == 0 )
   {
      bool pointAtSet( false ), falloffSet( false ), radiusSet( false ), tightnessSet( false ),
           distanceSet( false ), powerSet( false ), shadowless( false );
      povray::SpotLight spotlight( gpos, color );

      // Extracting the spotlight parameters
      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracing the focus point of the spotlight
         if( word_.compare( "point_at" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate point_at command
            if( pointAtSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'point_at' command!\n";
            }
            else pointAtSet = true;

            // Extracting the focus point
            Vec3 pointAt;

            if( !(input_ >> pointAt) ) {
               error << "   Line " << commandLine_ << ": Invalid 'point_at' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::PointAt( pointAt ) );
         }

         // Extracting the falloff setting
         else if( word_.compare( "falloff" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( falloffSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'falloff' command!\n";
            }
            else falloffSet = true;

            // Extracting the falloff value
            real falloff( 0 );

            if( !(input_ >> falloff) || falloff < real(0) || falloff > M_PI/real(2) ) {
               error << "   Line " << commandLine_ << ": Invalid 'falloff' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::Falloff( falloff ) );
         }

         // Extracing the radius setting
         else if( word_.compare( "radius" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( radiusSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'radius' command!\n";
            }
            else radiusSet = true;

            // Extracting the radius value
            real radius( 0 );

            if( !(input_ >> radius) || radius < real(0) || radius > M_PI/real(2) ) {
               error << "   Line " << commandLine_ << ": Invalid 'radius' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::Radius( radius ) );
         }

         // Extracting the tightness setting
         else if( word_.compare( "tightness" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( tightnessSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'tightness' command!\n";
            }
            else tightnessSet = true;

            // Extracting the tightness value
            real tightness( 0 );

            if( !(input_ >> tightness) || tightness < real(0) || tightness > real(100) ) {
               error << "   Line " << commandLine_ << ": Invalid 'tightness' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::Tightness( tightness ) );
         }

         // Extracting the light fading distance
         else if( word_.compare( "fade_distance" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( distanceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_distance' command!\n";
            }
            else distanceSet = true;

            // Extracting the fading distance
            real distance( 0 );

            if( !(input_ >> distance) || distance <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_distance' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::FadeDistance( distance ) );
         }

         // Extracting the light fading power
         else if( word_.compare( "fade_power" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_power command
            if( powerSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_power' command!\n";
            }
            else powerSet = true;

            // Extracting the fading power
            real power( 0 );

            if( !(input_ >> power) || power <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_power' command!\n";
               errorMode = true;
               input_.clear();
            }
            else spotlight.add( povray::FadePower( power ) );
         }

         // Extracting the shadowless keyword
         else if( word_.compare( "shadowless" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate shadowless command
            if( shadowless ) {
               error << "   Line " << commandLine_ << ": Duplicate 'shadowless' command!\n";
            }
            else shadowless = true;

            spotlight.add( povray::Shadowless() );
         }

         // Ending the 'light_source' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'light_source' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      lightsource = spotlight;
   }

   // Extracting a parallel light source
   else if( word_.compare( "parallel" ) == 0 )
   {
      bool distanceSet( false ), powerSet( false ), pointAtSet( false ), shadowless( false );
      povray::ParallelLight parallelLight( gpos, color );

      // Extracting the parallel light parameters
      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the light fading distance
         if( word_.compare( "fade_distance" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( distanceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_distance' command!\n";
            }
            else distanceSet = true;

            // Extracting the fading distance
            real distance( 0 );

            if( !(input_ >> distance) || distance <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_distance' command!\n";
               errorMode = true;
               input_.clear();
            }
            else parallelLight.add( povray::FadeDistance( distance ) );
         }

         // Extracting the light fading power
         else if( word_.compare( "fade_power" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_power command
            if( powerSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_power' command!\n";
            }
            else powerSet = true;

            // Extracting the fading power
            real power( 0 );

            if( !(input_ >> power) || power <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_power' command!\n";
               errorMode = true;
               input_.clear();
            }
            else parallelLight.add( povray::FadePower( power ) );
         }

         // Extracing the focus point of the light source
         else if( word_.compare( "point_at" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate point_at command
            if( pointAtSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'point_at' command!\n";
            }
            else pointAtSet = true;

            // Extracting the focus point
            Vec3 pointAt;

            if( !(input_ >> pointAt) ) {
               error << "   Line " << commandLine_ << ": Invalid 'point_at' command!\n";
               errorMode = true;
               input_.clear();
            }
            else parallelLight.add( povray::PointAt( pointAt ) );
         }

         // Extracting the shadowless keyword
         else if( word_.compare( "shadowless" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate shadowless command
            if( shadowless ) {
               error << "   Line " << commandLine_ << ": Duplicate 'shadowless' command!\n";
            }
            else shadowless = true;

            parallelLight.add( povray::Shadowless() );
         }

         // Ending the 'light_source' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'light_source' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      lightsource = parallelLight;
   }

   // Extracting an area light source
   else if( word_.compare( "area_light" ) == 0 )
   {
      bool adaptiveSet( false ), distanceSet( false ), powerSet( false ), shadowless( false );
      UnsignedValue<size_t> m, n;
      Vec3 u, v;

      // Extracting the first axis
      commandLine_ = getLineNumber();
      if( !(input_ >> u) ) {
         error << "   Line " << commandLine_ << ": Invalid first axis for 'area_light'!\n";
         input_.clear();
      }

      // Extracting the second axis
      commandLine_ = getLineNumber();
      if( !(input_ >> v) ) {
         error << "   Line " << commandLine_ << ": Invalid second axis for 'area_light'!\n";
         input_.clear();
      }

      // Extracting the number of light sources along the first axis
      commandLine_ = getLineNumber();
      if( !(input_ >> m) || m == 0 ) {
         error << "   Line " << commandLine_ << ": Invalid number of light sources on the first axis!\n";
         input_.clear();
      }

      // Extracting the number of light sources along the second axis
      commandLine_ = getLineNumber();
      if( !(input_ >> n) || n == 0 ) {
         error << "   Line " << commandLine_ << ": Invalid number of light sources on the second axis!\n";
         input_.clear();
      }

      // Creating a new area light source
      povray::AreaLight areaLight( gpos, color, u, v, m, n );

      // Extracing the area light parameters
      while( input_ >> word_ )
      {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the adaptive sampling level
         if( word_.compare( "adaptive" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate adaptive command
            if( adaptiveSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'adaptive' command!\n";
            }
            else adaptiveSet = true;

            // Extracting the adaptive sampling level
            UnsignedValue<unsigned int> adaptive( 0 );

            if( !(input_ >> adaptive) ) {
               error << "   Line " << commandLine_ << ": Invalid 'adaptive' command!\n";
               errorMode = true;
               input_.clear();
            }
            else areaLight.add( povray::Adaptive( adaptive ) );
         }

         // Extracting the light fading distance
         else if( word_.compare( "fade_distance" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( distanceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_distance' command!\n";
            }
            else distanceSet = true;

            // Extracting the fading distance
            real distance( 0 );

            if( !(input_ >> distance) || distance <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_distance' command!\n";
               errorMode = true;
               input_.clear();
            }
            else areaLight.add( povray::FadeDistance( distance ) );
         }

         // Extracting the light fading power
         else if( word_.compare( "fade_power" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_power command
            if( powerSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_power' command!\n";
            }
            else powerSet = true;

            // Extracting the fading power
            real power( 0 );

            if( !(input_ >> power) || power <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_power' command!\n";
               errorMode = true;
               input_.clear();
            }
            else areaLight.add( povray::FadePower( power ) );
         }

         // Extracting the shadowless keyword
         else if( word_.compare( "shadowless" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate shadowless command
            if( shadowless ) {
               error << "   Line " << commandLine_ << ": Duplicate 'shadowless' command!\n";
            }
            else shadowless = true;

            areaLight.add( povray::Shadowless() );
         }

         // Ending the 'light_source' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'light_source' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }

      lightsource = areaLight;
   }

   // Extracing a point light source
   else {
      bool distanceSet( false ), powerSet( false ), shadowless( false );
      povray::PointLight pointLight( gpos, color );

      // Extracting the point light parameters
      do {
         // Calculating the line number
         commandLine_ = getLineNumber();

         // Extracting the light fading distance
         if( word_.compare( "fade_distance" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_distance command
            if( distanceSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_distance' command!\n";
            }
            else distanceSet = true;

            // Extracting the fading distance
            real distance( 0 );

            if( !(input_ >> distance) || distance <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_distance' command!\n";
               errorMode = true;
               input_.clear();
            }
            else pointLight.add( povray::FadeDistance( distance ) );
         }

         // Extracting the light fading power
         else if( word_.compare( "fade_power" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate fade_power command
            if( powerSet ) {
               error << "   Line " << commandLine_ << ": Duplicate 'fade_power' command!\n";
            }
            else powerSet = true;

            // Extracting the fading power
            real power( 0 );

            if( !(input_ >> power) || power <= real(0) ) {
               error << "   Line " << commandLine_ << ": Invalid 'fade_power' command!\n";
               errorMode = true;
               input_.clear();
            }
            else pointLight.add( povray::FadePower( power ) );
         }

         // Extracting the 'shadowless' keyword
         else if( word_.compare( "shadowless" ) == 0 )
         {
            errorMode = false;

            // Detecting a duplicate shadowless command
            if( shadowless ) {
               error << "   Line " << commandLine_ << ": Duplicate 'shadowless' command!\n";
            }
            else shadowless = true;

            pointLight.add( povray::Shadowless() );
         }

         // Ending the 'light_source' command
         else if( word_.compare( "}" ) == 0 )
         {
            endTagFound = true;
            break;
         }

         // Treatment of unknown options in error mode
         else if( errorMode ) continue;

         // Treatment of unknown options
         else
         {
            error << "   Line " << commandLine_ << ": Unknown 'light_source' option: '" << word_ << "'!\n";
            errorMode = true;
         }
      }
      while( input_ >> word_ );

      lightsource = pointLight;
   }

   // Checking, whether the trailing bracket was found
   if( !endTagFound ) {
      error << "   '}' expected after 'light_source' section starting in line " << lightsourceLine << "!\n";
   }

   return !error;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the entire simulation world.
 *
 * \param filename The rigid body parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of all rigid bodies contained in the simulation world to
 * the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the rigid body parameters to an existing file
 * choose \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   ConstWorldID world = theWorld();

   // Writing all spheres to the parameter file
   {
      const World::Bodies::ConstCastIterator<Sphere> end( world->end<Sphere>() );
      for( World::Bodies::ConstCastIterator<Sphere> s=world->begin<Sphere>(); s!=end; ++s )
         writeFile( out, *s );
   }

   // Writing all boxes to the parameter file
   {
      const World::Bodies::ConstCastIterator<Box> end( world->end<Box>() );
      for( World::Bodies::ConstCastIterator<Box> b=world->begin<Box>(); b!=end; ++b )
         writeFile( out, *b );
   }

   // Writing all capsules to the parameter file
   {
      const World::Bodies::ConstCastIterator<Capsule> end( world->end<Capsule>() );
      for( World::Bodies::ConstCastIterator<Capsule> c=world->begin<Capsule>(); c!=end; ++c )
         writeFile( out, *c );
   }

   // Writing all cylinders to the parameter file
   {
      const World::Bodies::ConstCastIterator<Cylinder> end( world->end<Cylinder>() );
      for( World::Bodies::ConstCastIterator<Cylinder> c=world->begin<Cylinder>(); c!=end; ++c )
         writeFile( out, *c );
   }

   // Writing all planes to the parameter file
   {
      const World::Bodies::ConstCastIterator<Plane> end( world->end<Plane>() );
      for( World::Bodies::ConstCastIterator<Plane> p=world->begin<Plane>(); p!=end; ++p )
         writeFile( out, *p );
   }

   // Writing all unions to the parameter file
   {
      const World::Bodies::ConstCastIterator<Union> end( world->end<Union>() );
      for( World::Bodies::ConstCastIterator<Union> u=world->begin<Union>(); u!=end; ++u )
         writeFile( out, *u );
   }

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given sphere.
 *
 * \param filename The rigid body parameter file.
 * \param sphere The sphere to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given sphere to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the sphere to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstSphereID sphere,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the sphere to the parameter file
   writeFile( out, sphere );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given box.
 *
 * \param filename The rigid body parameter file.
 * \param box The box to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given box to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the box to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstBoxID box,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the box to the parameter file
   writeFile( out, box );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given capsule.
 *
 * \param filename The rigid body parameter file.
 * \param capsule The capsule to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given capsule to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the capsule to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstCapsuleID capsule,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the capsule to the parameter file
   writeFile( out, capsule );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given cylinder.
 *
 * \param filename The rigid body parameter file.
 * \param cylinder The cylinder to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given cylinder to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the cylinder to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstCylinderID cylinder,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the cylinder to the parameter file
   writeFile( out, cylinder );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given plane.
 *
 * \param filename The rigid body parameter file.
 * \param plane The plane to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given plane to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the plane to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstPlaneID plane,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the plane to the parameter file
   writeFile( out, plane );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a parameter file for the given union.
 *
 * \param filename The rigid body parameter file.
 * \param u The union to be written to the parameter file.
 * \param mode The mode of the output stream.
 * \return void
 *
 * This function writes the parameters of the given union to the parameter file \a filename.\n
 * The default mode for writing the parameter file is to replace all data contained in the file
 * (\a std::ofstream::trunc). In order to append the union to an existing file choose
 * \a std::ofstream::app instead.
 */
void BodyReader::writeFile( const char* const filename, ConstUnionID u,
                            std::ofstream::openmode mode )
{
   // Opening the parameter file
   mode |= std::ofstream::out;
   std::ofstream out( filename, mode );
   if( !out.is_open() ) {
      error_ << "   Error opening parameter output file '" << filename << "' !\n";
      return;
   }

   // Writing the union to the parameter file
   writeFile( out, u );

   // Closing the parameter file
   out.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given sphere.
 *
 * \param os Reference to the output stream.
 * \param sphere The sphere to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstSphereID sphere ) const
{
   // Don't write remote object
   if( sphere->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "sphere {\n"
      << "   id " << sphere->getID() << "\n"
      << "   label " << sphere->getSystemID() << "\n"
      << "   center " << sphere->getPosition() << "\n"
      << "   radius " << sphere->getRadius() << "\n"
      << "   material " << Material::getName( sphere->getMaterial() ) << "\n"
      << "   linear " << sphere->getLinearVel() << "\n"
      << "   angular " << sphere->getAngularVel() << "\n"
      << "   rotate " << sphere->getRotation().getEulerAnglesXYZ() << "\n";

   if(  sphere->isFixed()   ) os << "   fixed\n";
   if( !sphere->isVisible() ) os << "   invisible\n";

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given box.
 *
 * \param os Reference to the output stream.
 * \param box The box to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstBoxID box ) const
{
   // Don't write remote object
   if( box->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "box {\n"
      << "   id " << box->getID() << "\n"
      << "   label " << box->getSystemID() << "\n"
      << "   center " << box->getPosition() << "\n"
      << "   lengths " << box->getLengths() << "\n"
      << "   material " << Material::getName( box->getMaterial() ) << "\n"
      << "   linear " << box->getLinearVel() << "\n"
      << "   angular " << box->getAngularVel() << "\n"
      << "   rotate " << box->getRotation().getEulerAnglesXYZ() << "\n";

   if(  box->isFixed()   ) os << "   fixed\n";
   if( !box->isVisible() ) os << "   invisible\n";

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given capsule.
 *
 * \param os Reference to the output stream.
 * \param capsule The capsule to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstCapsuleID capsule ) const
{
   // Don't write remote object
   if( capsule->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "capsule {\n"
      << "   id " << capsule->getID() << "\n"
      << "   label " << capsule->getSystemID() << "\n"
      << "   center " << capsule->getPosition() << "\n"
      << "   radius " << capsule->getRadius() << "\n"
      << "   length " << capsule->getLength() << "\n"
      << "   material " << Material::getName( capsule->getMaterial() ) << "\n"
      << "   linear " << capsule->getLinearVel() << "\n"
      << "   angular " << capsule->getAngularVel() << "\n"
      << "   rotate " << capsule->getRotation().getEulerAnglesXYZ() << "\n";

   if(  capsule->isFixed()   ) os << "   fixed\n";
   if( !capsule->isVisible() ) os << "   invisible\n";

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given cylinder.
 *
 * \param os Reference to the output stream.
 * \param cylinder The cylinder to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstCylinderID cylinder ) const
{
   // Don't write remote object
   if( cylinder->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "cylinder {\n"
      << "   id " << cylinder->getID() << "\n"
      << "   label " << cylinder->getSystemID() << "\n"
      << "   center " << cylinder->getPosition() << "\n"
      << "   radius " << cylinder->getRadius() << "\n"
      << "   length " << cylinder->getLength() << "\n"
      << "   material " << Material::getName( cylinder->getMaterial() ) << "\n"
      << "   linear " << cylinder->getLinearVel() << "\n"
      << "   angular " << cylinder->getAngularVel() << "\n"
      << "   rotate " << cylinder->getRotation().getEulerAnglesXYZ() << "\n";

   if(  cylinder->isFixed()   ) os << "   fixed\n";
   if( !cylinder->isVisible() ) os << "   invisible\n";

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given plane.
 *
 * \param os Reference to the output stream.
 * \param plane The plane to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstPlaneID plane ) const
{
   // Don't write remote object
   if( plane->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "plane {\n"
      << "   id " << plane->getID() << "\n"
      << "   label " << plane->getSystemID() << "\n"
      << "   normal " << plane->getNormal() << "\n"
      << "   displacement " << plane->getDisplacement() << "\n"
      << "   material " << Material::getName( plane->getMaterial() ) << "\n";

   if( !plane->isVisible() ) os << "   invisible\n";

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the parameters of the given union.
 *
 * \param os Reference to the output stream.
 * \param u The union to be written to the parameter file.
 * \return void
 */
void BodyReader::writeFile( std::ostream& os, ConstUnionID u ) const
{
   // Don't write remote object
   if( u->isRemote() ) return;

   os << "//*************************************************************************************************\n"
      << "union {\n"
      << "   id " << u->getID() << "\n"
      << "   label " << u->getSystemID() << "\n"
      << "   center " << u->getPosition() << "\n"
      << "   linear " << u->getLinearVel() << "\n"
      << "   angular " << u->getAngularVel() << "\n";

   for( Union::ConstCastIterator<Sphere> s=u->begin<Sphere>(); s!=u->end<Sphere>(); ++s )
   {
      os << "   sphere {\n"
         << "      id " << s->getID() << "\n"
         << "      label " << s->getSystemID() << "\n"
         << "      center " << s->getPosition() << "\n"
         << "      radius " << s->getRadius() << "\n"
         << "      material " << Material::getName( s->getMaterial() ) << "\n"
         << "      rotate " << s->getRotation().getEulerAnglesXYZ() << "\n"
         << "   }\n";
   }

   for( Union::ConstCastIterator<Box> b=u->begin<Box>(); b!=u->end<Box>(); ++b )
   {
      os << "   box {\n"
         << "      id " << b->getID() << "\n"
         << "      label " << b->getSystemID() << "\n"
         << "      center " << b->getPosition() << "\n"
         << "      lengths " << b->getLengths() << "\n"
         << "      material " << Material::getName( b->getMaterial() ) << "\n"
         << "      rotate " << b->getRotation().getEulerAnglesXYZ() << "\n"
         << "   }\n";
   }

   for( Union::ConstCastIterator<Cylinder> c=u->begin<Cylinder>(); c!=u->end<Cylinder>(); ++c )
   {
      os << "   cylinder {\n"
         << "      id " << c->getID() << "\n"
         << "      label " << c->getSystemID() << "\n"
         << "      center " << c->getPosition() << "\n"
         << "      radius " << c->getRadius() << "\n"
         << "      length " << c->getLength() << "\n"
         << "      material " << Material::getName( c->getMaterial() ) << "\n"
         << "      rotate " << c->getRotation().getEulerAnglesXYZ() << "\n"
         << "   }\n";
   }

   for( Union::ConstCastIterator<Plane> p=u->begin<Plane>(); p!=u->end<Plane>(); ++p )
   {
      os << "   plane {\n"
         << "      id " << p->getID() << "\n"
         << "      label " << p->getSystemID() << "\n"
         << "      normal " << p->getNormal() << "\n"
         << "      displacement " << p->getDisplacement() << "\n"
         << "      material " << Material::getName( p->getMaterial() ) << "\n"
         << "      rotate " << p->getRotation().getEulerAnglesXYZ() << "\n"
         << "   }\n";
   }

   for( Union::ConstLinkIterator l=u->beginLinks(); l!=u->endLinks(); ++l )
   {
      ConstBodyID b1( l->getBody1() );
      ConstBodyID b2( l->getBody2() );

      os << "   link {\n"
         << "      id " << l->getID() << "\n"
         << "      body1 " << b1->getSystemID() << "\n"
         << "      body2 " << b2->getSystemID() << "\n"
         << "   }\n";
   }

   os << "}\n"
      << "//*************************************************************************************************"
      << "\n\n\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Estimating the input file line depending on the stream position of the preprocessed stream.
 *
 * \return The line number of the input file.
 */
size_t BodyReader::getLineNumber()
{
   const sstreamPos pos( input_.tellg() );

   lineVector::const_iterator it=lineNumbers_.begin();
   for( ; it!=lineNumbers_.end()-1; ++it ) {
      if( (it+1)->first > pos )
         return it->second;
   }
   return it->second;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finishing the current parameter block.
 *
 * \return void
 *
 * The function finishes the current parameter block. It searches the trailing bracket of the
 * parameter block and takes all blocks within the block into account. In contrast to the
 * BodyReader::skipBlock function, any leading bracket is considered to open a new block
 * within the current parameter block.
 */
void BodyReader::finishBlock()
{
   unsigned int counter(1);

   while( input_ >> word_ ) {
      if( word_.compare( "}" ) == 0 && --counter == 0 ) break;
      else if( word_.compare( "{" ) == 0 ) ++counter;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Skipping a parameter block.
 *
 * \return void
 *
 * The function skips the following parameter block of the type

   \code
   block { ... }
   \endcode

 * The function searches the trailing bracket of the parameter block and takes all blocks within
 * the block into account. In contrast to the BodyReader::finishBlock function, in case of an
 * immediate leading bracket the bracket is considered to be the leading bracket of the following
 * parameter block. Therefore both cases that the leading bracket has been extracted and has not
 * been extracted from the input stream (or has been forgotten) are handled.
 */
void BodyReader::skipBlock()
{
   unsigned int counter(1);
   LeadingBracket leadingbracket;

   if( !(input_ >> leadingbracket) )
      input_.clear();

   while( input_ >> word_ ) {
      if( word_.compare( "}" ) == 0 && --counter == 0 ) break;
      else if( word_.compare( "{" ) == 0 ) ++counter;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::ERROR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::Error class.
 */
BodyReader::Error::Error()
   : error_(false)
   , message_()
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::SphereParameters class.
 *
 * \param error The Error object to be copied.
 */
BodyReader::Error::Error( const Error& error )
   : error_( error.error_ )
   , message_( error.message_.str() )
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::Error class.
 */
BodyReader::Error::~Error()
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy assignment operator for the BodyReader::Error class.
 *
 * \param error The Error object to be copied.
 * \return Reference to the assigned Error object.
 */
BodyReader::Error& BodyReader::Error::operator=( const Error& error )
{
   if( &error == this ) return *this;
   error_ = error.error_;
   message_.str( error.message_.str() );
   return *this;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SPHEREPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::SphereParameters class.
 */
BodyReader::SphereParameters::SphereParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag
   , line(0)                     // Line number of the sphere in the parameter file
   , id(0)                       // User-specific ID of the sphere
   , label()                     // Unique label
   , material(invalid_material)  // Material of the sphere
   , radius(0)                   // Radius of the sphere
   , center()                    // Global position of the center of mass
   , linear()                    // Global linear velocity of the sphere
   , angular()                   // Global angular velocity of the sphere
   , translation()               // Total translation of the sphere
   , rotations()                 // Specified sphere rotations
   , texture()                   // The POV-Ray texture of the sphere
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::SphereParameters class.
 *
 * \param sp The SphereParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::SphereParameters::SphereParameters( const SphereParameters& sp )
   : fixed(sp.fixed)              // Fixation flag
   , visible(sp.visible)          // Visibility flag
   , materialSet(sp.materialSet)  // Material flag
   , textureSet(sp.textureSet)    // POV-Ray texture flag
   , line(sp.line)                // Line number of the sphere in the parameter file
   , id(sp.id)                    // User-specific ID of the sphere
   , label(sp.label)              // Unique label
   , material(sp.material)        // Material of the sphere
   , radius(sp.radius)            // Radius of the sphere
   , center(sp.center)            // Global position of the center of mass
   , linear(sp.linear)            // Global linear velocity of the sphere
   , angular(sp.angular)          // Global angular velocity of the sphere
   , translation(sp.translation)  // Total translation of the sphere
   , rotations(sp.rotations)      // Specified sphere rotations
   , texture(sp.texture)          // The POV-Ray texture of the sphere
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::SphereParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::SphereParameters::~SphereParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::BOXPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::BoxParameters class.
 */
BodyReader::BoxParameters::BoxParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag
   , line(0)                     // Line number of the box in the parameter file
   , id(0)                       // User-specific ID of the box
   , label()                     // Unique label
   , material(invalid_material)  // Material of the box
   , center()                    // Global position of the center of mass
   , lengths()                   // Side lengths of the box
   , linear()                    // Global linear velocity of the box
   , angular()                   // Global angular velocity of the box
   , translation()               // Total translation of the box
   , rotations()                 // Specified box rotations
   , texture()                   // The POV-Ray texture of the box
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::BoxParameters class.
 *
 * \param bp The BoxParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::BoxParameters::BoxParameters( const BoxParameters& bp )
   : fixed(bp.fixed)              // Fixation flag
   , visible(bp.visible)          // Visibility flag
   , materialSet(bp.materialSet)  // Material flag
   , textureSet(bp.textureSet)    // POV-Ray texture flag
   , line(bp.line)                // Line number of the box in the parameter file
   , id(bp.id)                    // User-specific ID of the box
   , label(bp.label)              // Unique label
   , material(bp.material)        // Material of the box
   , center(bp.center)            // Global position of the center of mass
   , lengths(bp.lengths)          // Side lengths of the box
   , linear(bp.linear)            // Global linear velocity of the box
   , angular(bp.angular)          // Global angular velocity of the box
   , translation(bp.translation)  // Total translation of the box
   , rotations(bp.rotations)      // Specified box rotations
   , texture(bp.texture)          // The POV-Ray texture of the box
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::BoxParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::BoxParameters::~BoxParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::CAPSULEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::CapsuleParameters class.
 */
BodyReader::CapsuleParameters::CapsuleParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag
   , line(0)                     // Line number of the capsule in the parameter file
   , id(0)                       // User-specific ID of the capsule
   , label()                     // Unique label
   , material(invalid_material)  // Material of the capsule
   , radius(0)                   // Radius of the capsule
   , length(0)                   // Length of the capsule
   , center()                    // Global position of the center of mass
   , linear()                    // Global linear velocity of the capsule
   , angular()                   // Global angular velocity of the capsule
   , translation()               // Total translation of the capsule
   , rotations()                 // Specified capsule rotations
   , texture()                   // The POV-Ray texture of the capsule
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::CapsuleParameters class.
 *
 * \param cp The CapsuleParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::CapsuleParameters::CapsuleParameters( const CapsuleParameters& cp )
   : fixed(cp.fixed)              // Fixation flag
   , visible(cp.visible)          // Visibility flag
   , materialSet(cp.materialSet)  // Material flag
   , textureSet(cp.textureSet)    // POV-Ray texture flag
   , line(cp.line)                // Line number of the capsule in the parameter file
   , id(cp.id)                    // User-specific ID of the capsule
   , label(cp.label)              // Unique label
   , material(cp.material)        // Material of the capsule
   , radius(cp.radius)            // Radius of the capsule
   , length(cp.length)            // Length of the capsule
   , center(cp.center)            // Global position of the center of mass
   , linear(cp.linear)            // Global linear velocity of the capsule
   , angular(cp.angular)          // Global angular velocity of the capsule
   , translation(cp.translation)  // Total translation of the capsule
   , rotations(cp.rotations)      // Specified capsule rotations
   , texture(cp.texture)          // The POV-Ray texture of the capsule
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::CapsuleParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::CapsuleParameters::~CapsuleParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::CYLINDERPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::CylinderParameters class.
 */
BodyReader::CylinderParameters::CylinderParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag
   , line(0)                     // Line number of the cylinder in the parameter file
   , id(0)                       // User-specific ID of the cylinder
   , label()                     // Unique label
   , material(invalid_material)  // Material of the cylinder
   , radius(0)                   // Radius of the cylinder
   , length(0)                   // Length of the cylinder
   , center()                    // Global position of the center of mass
   , linear()                    // Global linear velocity of the cylinder
   , angular()                   // Global angular velocity of the cylinder
   , translation()               // Total translation of the cylinder
   , rotations()                 // Specified cylinder rotations
   , texture()                   // The POV-Ray texture of the cylinder
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::CylinderParameters class.
 *
 * \param cp The CylinderParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::CylinderParameters::CylinderParameters( const CylinderParameters& cp )
   : fixed(cp.fixed)              // Fixation flag
   , visible(cp.visible)          // Visibility flag
   , materialSet(cp.materialSet)  // Material flag
   , textureSet(cp.textureSet)    // POV-Ray texture flag
   , line(cp.line)                // Line number of the cylinder in the parameter file
   , id(cp.id)                    // User-specific ID of the cylinder
   , label(cp.label)              // Unique label
   , material(cp.material)        // Material of the cylinder
   , radius(cp.radius)            // Radius of the cylinder
   , length(cp.length)            // Length of the cylinder
   , center(cp.center)            // Global position of the center of mass
   , linear(cp.linear)            // Global linear velocity of the cylinder
   , angular(cp.angular)          // Global angular velocity of the cylinder
   , translation(cp.translation)  // Total translation of the cylinder
   , rotations(cp.rotations)      // Specified cylinder rotations
   , texture(cp.texture)          // The POV-Ray texture of the cylinder
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::CylinderParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::CylinderParameters::~CylinderParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::PLANEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::PlaneParameters class.
 */
BodyReader::PlaneParameters::PlaneParameters()
   : visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag
   , line(0)                     // Line number of the plane in the parameter file
   , id(0)                       // User-specific ID of the plane
   , label()                     // Unique label
   , material(invalid_material)  // Material of the plane
   , displacement(0)             // Plane displacement
   , normal()                    // Normal of the plane
   , translation()               // Total translation of the plane
   , rotations()                 // Specified plane rotations
   , texture()                   // The POV-Ray texture of the plane
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::PlaneParameters class.
 *
 * \param pp The PlaneParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::PlaneParameters::PlaneParameters( const PlaneParameters& pp )
   : visible(pp.visible)            // Visibility flag
   , materialSet(pp.materialSet)    // Material flag
   , textureSet(pp.textureSet)      // POV-Ray texture flag
   , line(pp.line)                  // Line number of the plane in the parameter file
   , id(pp.id)                      // User-specific ID of the plane
   , label(pp.label)                // Unique label
   , material(pp.material)          // Material of the plane
   , displacement(pp.displacement)  // Plane displacement
   , normal(pp.normal)              // Normal of the plane
   , translation(pp.translation)    // Total translation of the plane
   , rotations(pp.rotations)        // Specified plane rotations
   , texture(pp.texture)            // The POV-Ray texture of the plane
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::PlaneParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::PlaneParameters::~PlaneParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::LINKPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::LinkParameters class.
 */
BodyReader::LinkParameters::LinkParameters()
   : id(0)           // User-specific ID of the link
   , body1()         // Label of the first linked primitive
   , body2()         // Label of the second linked primitive
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Equality comparison between two LinkParameters objects.
 *
 * \param lhs The left hand side link parameters.
 * \param rhs The right hand side link parameters.
 * \return \a true if the two LinkParameters objects are equal, \a false if not.
 */
bool BodyReader::LinkParameters::operator==( const LinkParameters& rhs )
{
   return body1 == rhs.body1 && body2 == rhs.body2;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::UNIONPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::UnionParameters class.
 */
BodyReader::UnionParameters::UnionParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , centerSet(false)            // Center flag
   , textureSet(false)           // POV-Ray texture flag.
   , line(0)                     // Line number of the union in the parameter file
   , id(0)                       // User-specific ID of the union
   , label()                     // Unique label
   , material(invalid_material)  // Material of the union
   , center()                    // Global position of the center of mass
   , linear()                    // Global linear velocity of the union
   , angular()                   // Global angular velocity of the union
   , translation()               // Total translation of the union
   , rotations()                 // Specified union rotations
   , spheres()                   // Spheres within the union
   , boxes()                     // Boxes within the union
   , capsules()                  // Capsules within the union
   , cylinders()                 // Cylinders within the union
   , planes()                    // Planes within the union
   , links()                     // Links within the union
   , texture()                   // POV-Ray texture of the union
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::UnionParameters class.
 *
 * \param cp The UnionParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::UnionParameters::UnionParameters( const UnionParameters& cp )
   : fixed(cp.fixed)              // Fixation flag
   , visible(cp.visible)          // Visibility flag
   , materialSet(cp.materialSet)  // Material flag
   , centerSet(cp.centerSet)      // Center flag
   , textureSet(cp.textureSet)    // POV-Ray texture flag
   , line(cp.line)                // Line number of the union in the parameter file
   , id(cp.id)                    // User-specific ID of the union
   , label(cp.label)              // Unique label
   , material(cp.material)        // Material of the union
   , center(cp.center)            // Global position of the center of mass
   , linear(cp.linear)            // Global linear velocity of the union
   , angular(cp.angular)          // Global angular velocity of the union
   , translation(cp.translation)  // Total translation of the union
   , rotations(cp.rotations)      // Specified union rotations
   , spheres(cp.spheres)          // Spheres within the union
   , boxes(cp.boxes)              // Boxes within the union
   , capsules(cp.capsules)        // Capsules within the union
   , cylinders(cp.cylinders)      // Cylinders within the union
   , planes(cp.planes)            // Planes within the union
   , links(cp.links)              // Links within the union
   , texture(cp.texture)          // POV-Ray texture of the union
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::UnionParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::UnionParameters::~UnionParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::AGGLOMERATEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::AgglomerateParameters class.
 */
BodyReader::AgglomerateParameters::AgglomerateParameters()
   : fixed(false)                // Fixation flag
   , visible(true)               // Visibility flag
   , materialSet(false)          // Material flag
   , textureSet(false)           // POV-Ray texture flag.
   , line(0)                     // Line number of the agglomerate in the parameter file
   , id(0)                       // User-specific ID of the agglomerate
   , label()                     // Unique label
   , number(0)                   // Number of spheres contained in the agglomerate
   , material(invalid_material)  // Material of the agglomerate
   , radius(0)                   // The radius of the agglomerate
   , threshold(0)                // Degree of clustering of the contained spheres
   , center()                    // Global position of the center of mass
   , linear()                    // Global linear velocity of the agglomerate
   , angular()                   // Global angular velocity of the agglomerate
   , translation()               // Total translation of the agglomerate
   , rotations()                 // Specified agglomerate rotations
   , texture()                   // POV-Ray texture of the agglomerate
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::AgglomerateParameters class.
 *
 * \param cp The UnionParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::AgglomerateParameters::AgglomerateParameters( const AgglomerateParameters& ap )
   : fixed(ap.fixed)              // Fixation flag
   , visible(ap.visible)          // Visibility flag
   , materialSet(ap.materialSet)  // Material flag
   , textureSet(ap.textureSet)    // POV-Ray texture flag
   , line(ap.line)                // Line number of the agglomerate in the parameter file
   , id(ap.id)                    // User-specific ID of the agglomerate
   , label(ap.label)              // Unique label
   , number(ap.number)            // Number of spheres contained in the agglomerate
   , material(ap.material)        // Material of the agglomerate
   , radius(ap.radius)            // The radius of the agglomerate
   , threshold(ap.threshold)      // Degree of clustering of the contained spheres
   , center(ap.center)            // Global position of the center of mass
   , linear(ap.linear)            // Global linear velocity of the agglomerate
   , angular(ap.angular)          // Global angular velocity of the agglomerate
   , translation(ap.translation)  // Total translation of the agglomerate
   , rotations(ap.rotations)      // Specified agglomerate rotations
   , texture(ap.texture)          // POV-Ray texture of the agglomerate
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::AgglomerateParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::AgglomerateParameters::~AgglomerateParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SPRINGPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the BodyReader::SpringParameters class.
 */
BodyReader::SpringParameters::SpringParameters()
   : visible(true)   // Visibility flag
   , line(0)         // Line number of the spring in the parameter file
   , stiffness(0)    // The stiffness of the spring
   , damping(0)      // The damping factor of the spring
   , length(0)       // The length in non-deformed state
   , body1()         // Label of the first attached rigid body
   , body2()         // Label of the second attached rigid body
   , anchor1()       // The first body's anchor point in body relative coordinates
   , anchor2()       // The second body's anchor point in body relative coordinates
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Copy Constructor for the BodyReader::SpringParameters class.
 *
 * \param sp The SpringParameters object to be copied.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::SpringParameters::SpringParameters( const SpringParameters& sp )
   : visible(sp.visible)      // Visibility flag
   , line(sp.line)            // Line number of the spring in the parameter file
   , stiffness(sp.stiffness)  // The stiffness of the spring
   , damping(sp.damping)      // The damping factor of the spring
   , length(sp.length)        // The length in non-deformed state
   , body1(sp.body1)          // The label of the first attached rigid body
   , body2(sp.body2)          // The label of the second attached rigid body
   , anchor1(sp.anchor1)      // The first body's anchor point in body relative coordinates
   , anchor2(sp.anchor2)      // The second body's anchor point in body relative coordinates
{}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Destructor for the BodyReader::SpringParameters class.
 *
 * Explicit definition to avoid inline warnings!
 */
BodyReader::SpringParameters::~SpringParameters()
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::EQUALSIGN
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the BodyReader::EqualSign class.
 *
 * \param is Reference to the input stream.
 * \param es Reference to a EqualSign object.
 * \return The input stream.
 *
 * Input operator for a '=' sign. The next non-whitespace character in the input stream is
 * expected to be an equal sign. Otherwise the stream position is returned to the previous
 * position and the \a std::istream::failbit is set. Since \a EqualSign is used only within
 * the \a BodyReader, the input stream is assumed to have the default behavior.
 */
std::istream& operator>>( std::istream& is, BodyReader::EqualSign& /*es*/ )
{
   char c;
   const std::istream::pos_type pos( is.tellg() );

   if( !(is >> c) || c != '=' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SEMICOLON
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the BodyReader::Semicolon class.
 *
 * \param is Reference to the input stream.
 * \param s Reference to a Semicolon object.
 * \return The input stream.
 *
 * Input operator for a ';' sign. The next non-whitespace character in the input stream is
 * expected to be a semicolon. Otherwise the stream position is returned to the previous
 * position and the \a std::istream::failbit is set. Since \a Semicolon is used only within
 * the \a BodyReader, the input stream is assumed to have the default behavior.
 */
std::istream& operator>>( std::istream& is, BodyReader::Semicolon& /*s*/ )
{
   char c;
   const std::istream::pos_type pos( is.tellg() );

   if( !(is >> c) || c != ';' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::LEADINGBRACKET
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the BodyReader::LeadingBracket class.
 *
 * \param is Reference to the input stream.
 * \param lb Reference to a LeadingBracket object.
 * \return The input stream.
 *
 * Input operator for a '{' sign. The next non-whitespace character is expected to be a
 * '{' sign, otherwise the stream position is returned to the previous position and the
 * \a std::istream::failbit is set. Since \a LeadingBracket is used only within the
 * \a BodyReader, the input stream is assumed to have the default behavior.
 */
std::istream& operator>>( std::istream& is, BodyReader::LeadingBracket& /*lb*/ )
{
   char c;
   const std::istream::pos_type pos( is.tellg() );

   if( !(is >> c) || c != '{' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::TRAILINGBRACKET
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input operator for the BodyReader::LeadingBracket class.
 *
 * \param is Reference to the input stream.
 * \param tb Reference to a TrailingBracket object.
 * \return The input stream.
 *
 * Input operator for a '}' sign. The next non-whitespace character is expected to be a
 * '}' sign, otherwise the stream position is returned to the previous position and the
 * \a std::istream::failbit is set. Since \a TrailingBracket is used only within the
 * \a BodyReader, the input stream is assumed to have the default behavior.
 */
std::istream& operator>>( std::istream& is, BodyReader::TrailingBracket& /*tb*/ )
{
   char c;
   const std::istream::pos_type pos( is.tellg() );

   if( !(is >> c) || c != '}' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
   }

   return is;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe
