//=================================================================================================
/*!
 *  \file pe/povray/Writer.h
 *  \brief POV-Ray file writer for the POV-Ray visualization
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

#ifndef _PE_POVRAY_WRITER_H_
#define _PE_POVRAY_WRITER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <boost/filesystem/path.hpp>
#include <boost/thread/mutex.hpp>
#include <pe/core/Types.h>
#include <pe/core/Visualization.h>
#include <pe/math/Constants.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/Vector3.h>
#include <pe/povray/Camera.h>
#include <pe/povray/Color.h>
#include <pe/povray/ColorMap.h>
#include <pe/povray/Finish.h>
#include <pe/povray/LightSource.h>
#include <pe/povray/Normal.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/WriterID.h>
#include <pe/povray/Texture.h>
#include <pe/povray/TexturePolicy.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/DerivedFrom.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Dependency.h>
#include <pe/util/Types.h>
#include <pe/util/Vector.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_writer POV-Ray writer
 * \ingroup povray
 */
/*!\brief POV-Ray visualization writer.
 * \ingroup povray_writer
 *
 * The Writer class offers the functionality to create POV-Ray files for a ray tracing
 * visualization of the rigid body simulation.\n
 * In order to activate the POV-Ray visualization use the following function:

   \code
   pe::povray::WriterID pe::povray::activateWriter();
   \endcode

 * This function activates the POV-Ray writer and returns the handle to the active writer.
 * Subsequent calls of this function will only return the handle to the writer.
 */
class PE_PUBLIC Writer : public Visualization
             , private Dependency<logging::Logger>
             , private Dependency<Camera>
{
private:
   //**Declarations for nested structures**********************************************************
   struct Declaration;
   struct SphereData;
   struct BoxData;
   struct CapsuleData;
   struct CylinderData;
   struct PlaneData;
   struct MeshData;
   struct SpringData;
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef Vector<std::string>   Includes;      //!< Vector of included files.
   typedef Vector<Declaration>   Declarations;  //!< Vector for declared POV-Ray identifiers.
   typedef Vector<LightSource>   LightSources;  //!< Vector of light sources.
   typedef Vector<SphereData>    Spheres;       //!< Vector of textured spheres.
   typedef Vector<BoxData>       Boxes;         //!< Vector of textured boxes.
   typedef Vector<CapsuleData>   Capsules;      //!< Vector of textured capsules.
   typedef Vector<CylinderData>  Cylinders;     //!< Vector of textured cylinders.
   typedef Vector<PlaneData>     Planes;        //!< Vector of textured planes.
   typedef Vector<MeshData>      Meshes;        //!< Vector of textured triangle meshes.
   typedef Vector<SpringData>    Springs;       //!< Vector of visible springs.

   //! Texture policy for the automatic assignment of initial textures.
   typedef std::auto_ptr<TexturePolicy>  Policy;
   /*! \endcond */
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Writer();
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Writer();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t getSteps() const;
   inline size_t getFileCounter() const;
   //@}
   //**********************************************************************************************

   //**Setup functions*****************************************************************************
   /*!\name Setup functions */
   //@{
                          inline void include( const std::string& filename );
                          inline void declare( const std::string& key, const Color& color );
                          inline void declare( const std::string& key, const ColorMap& colormap );
                          inline void declare( const std::string& key, const Pigment& pigment );
                          inline void declare( const std::string& key, const Finish& finish );
                          inline void declare( const std::string& key, const Normal& normal );
                          inline void declare( const std::string& key, const Texture& texture );
   template< typename T > inline void declare( const std::string& key, const Vector3<T>& v );
                          inline void undeclare( const std::string& key );
                                 void setFilename( const boost::filesystem::path& filename );
                                 void setStart  ( size_t start );
                          inline void setEnd    ( size_t end );
                          inline void setSpacing( size_t spacing );
                                 void setSteps  ( size_t steps );
                                 void setFileCounter( size_t counter );
                          inline void setBackground( const Color& color );
                          inline void setBackground( real red, real green, real blue, real alpha=0.0 );
                          inline void setBackground( const std::string& color );
                          inline void addLightSource( const LightSource& lightsource );
                          inline void setDecorations( bool on );
   //@}
   //**********************************************************************************************

   //**Texture functions***************************************************************************
   /*!\name Texture functions */
   //@{
   template< typename T > void setTexturePolicy( const T& policy );
                          void setTexture( ConstSphereID sphere, const Texture& texture );
                          void setTexture( ConstBoxID box, const Texture& texture );
                          void setTexture( ConstCapsuleID capsule, const Texture& texture );
                          void setTexture( ConstCylinderID cylinder, const Texture& texture );
                          void setTexture( ConstPlaneID plane, const Texture& texture );
                          void setTexture( ConstTriangleMeshID mesh, const Texture& texture );
                          void setTexture( ConstUnionID u, const Texture& texture );
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void writeFile( const boost::filesystem::path& filename );
   void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

private:
   //**Add functions*******************************************************************************
   /*!\name Add functions */
   //@{
   virtual void addSphere  ( ConstSphereID       sphere   );
   virtual void addBox     ( ConstBoxID          box      );
   virtual void addCapsule ( ConstCapsuleID      capsule  );
   virtual void addCylinder( ConstCylinderID     cylinder );
   virtual void addPlane   ( ConstPlaneID        plane    );
   virtual void addMesh    ( ConstTriangleMeshID mesh     );
   virtual void addSpring  ( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Remove functions****************************************************************************
   /*!\name Remove functions */
   //@{
   virtual void removeSphere  ( ConstSphereID       sphere   );
   virtual void removeBox     ( ConstBoxID          box      );
   virtual void removeCapsule ( ConstCapsuleID      capsule  );
   virtual void removeCylinder( ConstCylinderID     cylinder );
   virtual void removePlane   ( ConstPlaneID        plane    );
   virtual void removeMesh    ( ConstTriangleMeshID mesh     );
   virtual void removeSpring  ( ConstSpringID       spring   );
   //@}
   //**********************************************************************************************

   //**Visualization functions*********************************************************************
   /*!\name Visualization functions */
   //@{
   virtual void trigger();
   //@}
   //**********************************************************************************************

public:
   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   static void writeSphere  ( std::ostream& os, ConstSphereID       sphere  , const Texture& texture );
   static void writeBox     ( std::ostream& os, ConstBoxID          box     , const Texture& texture );
   static void writeCapsule ( std::ostream& os, ConstCapsuleID      capsule , const Texture& texture );
   static void writeCylinder( std::ostream& os, ConstCylinderID     cylinder, const Texture& texture );
   static void writePlane   ( std::ostream& os, ConstPlaneID        plane   , const Texture& texture );
   static void writeMesh    ( std::ostream& os, ConstTriangleMeshID mesh    , const Texture& texture );
   static void writeSpring  ( std::ostream& os, ConstSpringID       spring                           );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static inline const Vec3 calcEulerAngles( const Rot3& R );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t start_;               //!< First visualized time step.
   size_t end_;                 //!< Last visualized time step.
   size_t spacing_;             //!< Spacing between two visualized time steps.
   size_t steps_;               //!< Time step counter between two time steps.
   size_t counter_;             //!< Internal file counter.
   std::string prefix_;         //!< Prefix of the file name for the POV-Ray files.
   std::string postfix_;        //!< Postfix of the file name for the POV-Ray files.
   Color background_;           //!< The background color for the visualization.
   Policy policy_;              //!< The active texture policy.
   Includes includes_;          //!< Included POV-Ray header files.
   Declarations declarations_;  //!< Declared POV-Ray identifiers.
   LightSources lightsources_;  //!< Light sources for the visualization
   bool decorations_;           //!< Controls output of camera, lights and background.
   Spheres spheres_;            //!< Registered spheres for the visualization.
   Boxes boxes_;                //!< Registered boxes for the visualization.
   Capsules capsules_;          //!< Registered capsules for the visualization.
   Cylinders cylinders_;        //!< Registered capsules for the visualization.
   Planes planes_;              //!< Registered planes for the visualization.
   Meshes meshes_;              //!< Registered triangle meshes for the visualization.
   Springs springs_;            //!< Registered springs for the visualization.

   static bool active_;                 //!< Active flag of the POV-Ray writer.
   static boost::mutex instanceMutex_;  //!< Synchronization mutex for access to the POV-Ray writer.
   //@}
   //**********************************************************************************************

   //**Private struct Declaration******************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A POV-Ray declaration. */
   struct Declaration
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline Declaration( const std::string& key, const std::string& declared );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      std::string key_;       //!< POV-Ray identifier for the declaration.
      std::string declared_;  //!< The content of the declaration.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct SphereData*******************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured sphere primitive. */
   struct SphereData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline SphereData( ConstSphereID sphere, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;          //!< Default flag.
                              /*!< A value of \a true indicates the sphere has not been
                                   assigned an individual texture. In this case the texture
                                   policy is applied to give the sphere an appearance. */
      ConstSphereID sphere_;  //!< Handle for the sphere.
      Texture texture_;       //!< The texture of the sphere.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct BoxData**********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured box primitive. */
   struct BoxData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline BoxData( ConstBoxID box, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;     //!< Default flag.
                         /*!< A value of \a true indicates the box has not been assigned an
                              individual texture. In this case the texture policy is applied
                              to give the box an appearance. */
      ConstBoxID box_;   //!< Handle for the box.
      Texture texture_;  //!< The texture of the box.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct CapsuleData******************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured capsule primitive. */
   struct CapsuleData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline CapsuleData( ConstCapsuleID capsule, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;            //!< Default flag.
                                /*!< A value of \a true indicates the capsule has not been
                                     assigned an individual texture. In this case the texture
                                     policy is applied to give the capsule an appearance. */
      ConstCapsuleID capsule_;  //!< Handle for the capsule.
      Texture texture_;         //!< The texture of the capsule.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct CylinderData*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured cylinder primitive. */
   struct CylinderData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline CylinderData( ConstCylinderID cylinder, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;              //!< Default flag.
                                  /*!< A value of \a true indicates the cylinder has not been
                                       assigned an individual texture. In this case the texture
                                       policy is applied to give the cylinder an appearance. */
      ConstCylinderID cylinder_;  //!< Handle for the cylinder.
      Texture texture_;           //!< The texture of the cylinder.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct PlaneData********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured plane primitive. */
   struct PlaneData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline PlaneData( ConstPlaneID plane, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;        //!< Default flag.
                            /*!< A value of \a true indicates the plane has not been
                                 assigned an individual texture. In this case the texture
                                 policy is applied to give the plane an appearance. */
      ConstPlaneID plane_;  //!< Handle for the plane.
      Texture texture_;     //!< The texture of the plane.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct MeshData*********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A textured triangle mesh primitive. */
   struct MeshData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline MeshData( ConstTriangleMeshID mesh, const Texture& texture );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool default_;              //!< Default flag.
                                  /*!< A value of \a true indicates the triangle mesh has not been
                                       assigned an individual texture. In this case the texture
                                       policy is applied to give the mesh an appearance. */
      ConstTriangleMeshID mesh_;  //!< Handle for the triangle mesh.
      Texture texture_;           //!< The texture of the triangle mesh.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private struct SpringData*******************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief A visible spring. */
   struct SpringData
   {
    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      explicit inline SpringData( ConstSpringID spring );
      // No explicitly declared copy constructor.
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      // No explicitly declared destructor.
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      // No explicitly declared copy assignment operator.
      //*******************************************************************************************

      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      ConstSpringID spring_;  //!< Handle for the spring.
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend bool     isActive();
   friend WriterID activateWriter();
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the number of time steps skipped since the last PovRay output.
 *
 * \return The number of time steps skipped since the last PovRay output.
 */
inline size_t Writer::getSteps() const
{
   return steps_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of filestime steps skipped since the last PovRay output.
 *
 * \return The number of time steps skipped since the last PovRay output.
 */
inline size_t Writer::getFileCounter() const
{
   return counter_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Including a POV-Ray header file.
 *
 * \param filename File name of the include file.
 * \return void
 */
inline void Writer::include( const std::string& filename )
{
   includes_.pushBack( filename );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new color value.
 *
 * \param key The POV-Ray identifier for the declared color value.
 * \param color The new color value.
 * \return void
 */
inline void Writer::declare( const std::string& key, const Color& color )
{
   std::ostringstream oss;
   oss << color << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new color map.
 *
 * \param key The POV-Ray identifier for the declared color map.
 * \param colormap The new color map.
 * \return void
 */
inline void Writer::declare( const std::string& key, const ColorMap& colormap )
{
   std::ostringstream oss;
   oss << colormap << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new POV-Ray pigment.
 *
 * \param key The POV-Ray identifier for the declared pigment.
 * \param pigment The new POV-Ray pigment.
 * \return void
 */
inline void Writer::declare( const std::string& key, const Pigment& pigment )
{
   std::ostringstream oss;
   oss << pigment << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new POV-Ray pigment.
 *
 * \param key The POV-Ray identifier for the declared finish.
 * \param finish The new POV-Ray finish.
 * \return void
 */
inline void Writer::declare( const std::string& key, const Finish& finish )
{
   std::stringstream oss;
   oss << finish << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new POV-Ray normal.
 *
 * \param key The POV-Ray identifier for the declared normal.
 * \param normal The new POV-Ray normal.
 * \return void
 */
inline void Writer::declare( const std::string& key, const Normal& normal )
{
   std::stringstream oss;
   oss << normal << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new POV-Ray texture.
 *
 * \param key The POV-Ray identifier for the declared texture.
 * \param texture The new POV-Ray texture.
 * \return void
 */
inline void Writer::declare( const std::string& key, const Texture& texture )
{
   std::stringstream oss;
   oss << texture << ";";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Declaring a new vector.
 *
 * \param key The POV-Ray identifier for the declared vector.
 * \param v The new POV-Ray vector.
 * \return void
 */
template< typename Type >
inline void Writer::declare( const std::string& key, const Vector3<Type>& v )
{
   std::stringstream oss;
   oss << "<" << v[0] << ", " << v[1] << ", " << v[2] << ">;";
   declarations_.pushBack( Declaration( key, oss.str() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removes all declaration matching a specified key.
 *
 * \param key The POV-Ray identifier.
 * \return void
 */
inline void Writer::undeclare( const std::string& key )
{
   for( Declarations::Iterator it = declarations_.begin(); it != declarations_.end(); ++it ) {
      if( it->key_ == key ) {
         declarations_.erase( it );
         break;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the number of the last visualized time step.
 *
 * \param end Number of the last visualized time step.
 */
inline void Writer::setEnd( size_t end )
{
   end_ = end;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the spacing of the POV-Ray visualization.
 *
 * \param spacing Spacing between two visualized time steps \f$ [1..\infty) \f$.
 * \exception std::invalid_argument Invalid spacing value.
 */
inline void Writer::setSpacing( size_t spacing )
{
   // Checking the spacing value
   if( spacing_ == 0 )
      throw std::invalid_argument( "Invalid spacing value" );

   spacing_ = spacing;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the background color for the visualization.
 *
 * \param color The background color for the visualization.
 * \return void
 *
 * This function sets the background color for all generated POV-Ray files. The default
 * background color is a solid white.
 */
inline void Writer::setBackground( const Color& color )
{
   background_ = color;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the background color for the visualization.
 *
 * \param red The red channel of the background color. Has to be in the range \f$ [0..1] \f$.
 * \param green The green channel of the background color. Has to be in the range \f$ [0..1] \f$.
 * \param blue The blue channel of the background color. Has to be in the range \f$ [0..1] \f$.
 * \param alpha The transparency of the background color. Has to be in the range \f$ [0..1] \f$.
 * \return void
 * \exception std::invalid_argument Invalid color value.
 *
 * This function sets the background color for all generated POV-Ray files to the color specified
 * by the three color channels \a red, \a green and \a blue and the transparency value \a alpha.
 * The given values have to be in the range \f$ [0..1] \f$, otherwise a \a std::invalid_argument
 * exception is thrown. The default for the transparency value is 0, which results in a solid,
 * non-transparent color. The default background color is a solid white.
 */
inline void Writer::setBackground( real red, real green, real blue, real alpha )
{
   background_ = Color( red, green, blue, alpha );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the background color for the visualization.
 *
 * \param color The input string containing the color parameters.
 * \return void
 * \exception std::invalid_argument Invalid color input string.
 *
 * This function sets the background color for all generated POV-Ray files to the color specified
 * by the given color parameter string. For details about the format of the color parameter string,
 * see the details of the Color class description.The default background color is a solid white.
 */
inline void Writer::setBackground( const std::string& color )
{
   background_ = Color( color );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adding a light source to the POV-Ray visualization.
 *
 * \param lightsource The new POV-Ray light source.
 * \return void
 */
inline void Writer::addLightSource( const LightSource& lightsource )
{
   lightsources_.pushBack( lightsource );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets whether decorations are written.
 *
 * \param on True if decoration information is written, false otherwise.
 * \return void
 *
 * Decorations are camera information, light sources and background.
 */
inline void Writer::setDecorations( bool on )
{
   decorations_ = on;
}
//*************************************************************************************************




//=================================================================================================
//
//  TEXTURE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the texture policy for the POV-Ray visualization.
 *
 * \param policy The new texture policy.
 * \return void
 *
 * This function sets the texture policy for the assignment of initial textures. Every time
 * a new rigid body is created it is per default assigned a texture according to the given
 * texture policy. For the individual configuration of a texture, the setTexture() functions
 * can be used.
 *
 * \b Note: The given texture policy of type \a T must be derived from the TexturePolicy
 * interface class. In case \a T is not a texture policy, a compile time error is created!
 */
template< typename T >
void Writer::setTexturePolicy( const T& policy )
{
   // Resetting the texture policy
   pe_CONSTRAINT_MUST_BE_DERIVED_FROM( T, TexturePolicy );
   policy_.reset( new T( policy ) );

   // Applying the new texture policy to all registered spheres with default textures
   for( Spheres::Iterator s=spheres_.begin(); s!=spheres_.end(); ++s ) {
      if( s->default_ ) s->texture_ = policy_->getTexture( s->sphere_ );
   }

   // Applying the new texture policy to all registered boxes with default textures
   for( Boxes::Iterator b=boxes_.begin(); b!=boxes_.end(); ++b ) {
      if( b->default_ ) b->texture_ = policy_->getTexture( b->box_ );
   }

   // Applying the new texture policy to all registered capsules with default textures
   for( Capsules::Iterator c=capsules_.begin(); c!=capsules_.end(); ++c ) {
      if( c->default_ ) c->texture_ = policy_->getTexture( c->capsule_ );
   }

   // Applying the new texture policy to all registered cylinders with default textures
   for( Cylinders::Iterator c=cylinders_.begin(); c!=cylinders_.end(); ++c ) {
      if( c->default_ ) c->texture_ = policy_->getTexture( c->cylinder_ );
   }

   // Applying the new texture policy to all registered meshes with default textures
   //TODO warum geht das nicht?
   /*
   for( Meshes::Iterator m=meshes_.begin(); m!=meshes_.end(); ++m ) {
      if( m->default_ ){
         //ConstBodyID id = m->mesh_;
         //m->texture_ = policy_->getTexture( id );
         m->texture_ = policy_->getTexture( m->mesh_ );
      }
   }*/

   // Applying the new texture policy to all registered planes with default textures
   for( Planes::Iterator p=planes_.begin(); p!=planes_.end(); ++p ) {
      if( p->default_ ) p->texture_ = policy_->getTexture( p->plane_ );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the Euler angles (in degree).
 *
 * \param R The rotation of a rigid body.
 * \return The Euler angles for a rotation order of x, y, z.
 *
 * This function calculates the required Euler angles for the POV-Ray visualization. It
 * performs the necessary transformation for the right-handed pe coordinate system to the
 * left-handed POV-Ray coordinate system and calculates the Euler angles in degrees.
 */
inline const Vec3 Writer::calcEulerAngles( const Rot3& R )
{
   static const real factor( real(180)/M_PI );

   // Calculation of the left-handed Euler angles ( = factor * R.getEulerAngles( XYZs ) )
   const real cy( std::sqrt( R[0]*R[0] + R[6]*R[6] ) );
   if( cy > real(16)*std::numeric_limits<real>::epsilon() ) {
      return factor * Vec3( -std::atan2(  R[4], R[5] )+M_PI,
                             std::atan2( -R[3], cy ),
                             std::atan2(  R[6], R[0] ) );
   }
   else {
      return factor * Vec3( -std::atan2( -R[8], R[7] )+M_PI,
                             std::atan2( -R[3], cy ),
                             real(0) );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::DECLARATION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::Declaration class.
 *
 * \param key The POV-Ray identifier of the declaration.
 * \param declared The POV-Ray representation of the declaration.
 */
inline Writer::Declaration::Declaration( const std::string& key, const std::string& declared )
   : key_(key)            // POV-Ray identifier for the declaration
   , declared_(declared)  // The content of the declaration
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::SPHEREDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::SphereData class.
 *
 * \param sphere The registered sphere.
 * \param texture The texture of the sphere.
 */
inline Writer::SphereData::SphereData( ConstSphereID sphere, const Texture& texture )
   : default_(true)     // Default flag
   , sphere_(sphere)    // Handle for the sphere
   , texture_(texture)  // Texture of the sphere
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::BOXDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::BoxData class.
 *
 * \param box The registered box.
 * \param texture The texture of the box.
 */
inline Writer::BoxData::BoxData( ConstBoxID box, const Texture& texture )
   : default_(true)     // Default flag
   , box_(box)          // Handle for the box
   , texture_(texture)  // Texture of the box
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::CAPSULEDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::CapsuleData class.
 *
 * \param capsule The registered capsule.
 * \param texture The texture of the capsule.
 */
inline Writer::CapsuleData::CapsuleData( ConstCapsuleID capsule, const Texture& texture )
   : default_(true)     // Default flag
   , capsule_(capsule)  // Handle for the capsule
   , texture_(texture)  // Texture of the capsule
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::CAPSULEDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::CylinderData class.
 *
 * \param cyöomder The registered cylinder.
 * \param texture The texture of the cylinder.
 */
inline Writer::CylinderData::CylinderData( ConstCylinderID cylinder, const Texture& texture )
   : default_(true)       // Default flag
   , cylinder_(cylinder)  // Handle for the cylinder
   , texture_(texture)    // Texture of the cylinder
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::PLANEDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::PlaneData class.
 *
 * \param plane The registered plane.
 * \param texture The texture of the plane.
 */
inline Writer::PlaneData::PlaneData( ConstPlaneID plane, const Texture& texture )
   : default_(true)     // Default flag
   , plane_(plane)      // Handle for the plane
   , texture_(texture)  // Texture of the plane
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::MESHDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::MeshData class.
 *
 * \param mesh The registered triangle mesh.
 * \param texture The texture of the triangle mesh.
 */
inline Writer::MeshData::MeshData( ConstTriangleMeshID mesh, const Texture& texture )
   : default_(true)     // Default flag
   , mesh_(mesh)        // Handle for the triangle mesh
   , texture_(texture)  // Texture of the triangle mesh
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS WRITER::SPRINGDATA
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constructor for the Writer::SpringData class.
 *
 * \param spring The registered spring.
 */
inline Writer::SpringData::SpringData( ConstSpringID spring )
   : spring_(spring)  // Handle for the spring
{}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY WRITER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name POV-Ray writer setup functions */
//@{
inline    bool     isActive();
PE_PUBLIC WriterID activateWriter();
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the POV-Ray visualization is active or not.
 * \ingroup povray_writer
 *
 * \return \a true if the POV-Ray visualization is active, \a false if not.
 */
inline bool isActive()
{
   return Writer::active_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name POV-Ray writer operators */
//@{
std::ostream& operator<<( std::ostream& os, const Writer& pov );
std::ostream& operator<<( std::ostream& os, const WriterID& pov );
std::ostream& operator<<( std::ostream& os, const ConstWriterID& pov );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
