//=================================================================================================
/*!
 *  \file pe/core/Thresholds.h
 *  \brief Numerical thresholds for the physics engine
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

#ifndef _PE_CORE_THRESHOLDS_H_
#define _PE_CORE_THRESHOLDS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Collection of numerical threshold values.
 * \ingroup core
 *
 * The Thresholds class defines numerical floating point thresholds for the pe physics engine.
 * The following thresholds can be used:
 *
 * - \b collisionThreshold: This threshold is used for the contact classification. It is used to
 *      separate between separating, resting and colliding contacts.
 * - \b contactThreshold: This threshold is used to tell whether two rigid bodies are in contact or
 *      not. If the distance between two bodies is smaller than this threshold, they are considered
 *      to be in contact with each other.
 * - \b restitutionThreshold: In case the relative velocity between two colliding rigid bodies is
 *      smaller than this threshold, a coefficient of restitution of 0 is used to avoid an infinite
 *      number of collisions during a single time step.
 * - \b frictionThreshold: This threshold represents the boundary between static and dynamic
 *      friction. In case the relative tangential velocity of two contacting rigid bodies is
 *      smaller than this threshold, static friction is applied, else dynamic friction is used.
 * - \b surfaceThreshold: This threshold is used for surface checks. Only points with a distance
 *      to the surface smaller than this threshold are considered surface point.
 * - \b parallelThreshold: This threshold is used for parallelism checks. If the scalar product
 *      of two vectors is smaller than this threshold the vectors are considered to be parallel.
 *
 * The Thresholds class is used in the following manner:

   \code
   const double acc = Thresholds<double>::accuracy();
   if( dist < Thresholds<double>::surfaceThreshold ) {...}
   \endcode

 * \b Note: The Thresholds class is not defined for integral data types.
 */
template< typename Type >
struct Thresholds
{};
//*************************************************************************************************




//=================================================================================================
//
//  FLOAT SPECIALIZATION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Thresholds<float> specialization.
 * \ingroup core
 */
template<>
struct Thresholds<float>
{
public:
   //! Threshold for the contact classification.
   /*! This threshold separates between separating, resting and colliding contacts. */
   static inline float collisionThreshold() { return 1E-8F; }

   //! Threshold for the distance between two rigid bodies.
   /*! Rigid bodies with a distance smaller than this threshold are in contact. */
   static inline float contactThreshold() { return 5E-7F; }

   //! Threshold for the restriction of the coefficient of restitution.
   /*! In case the relative velocity between two colliding rigid bodies is smaller than this
       threshold, a coefficient of restitution of 0 is used to avoid an infinite number of
       collisions during a single time step. */
   static inline float restitutionThreshold() { return 1E-8F; }

   //! Threshold for the separation between static and dynamic friction.
   /*! This threshold represents the boundary between static and dynamic friction. */
   static inline float frictionThreshold() { return 1E-8F; }

   //! Threshold for surface points/checks.
   /*! Only points with a distance to the surface smaller than this threshold are considered
       surface point. */
   static inline float surfaceThreshold() { return 5E-7F; }

   //! Threshold for parallelism checks.
   /*! Scalar products smaller than this threshold value indicate parallel vectors. */
   static inline float parallelThreshold() { return 1E-8F; }
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  DOUBLE SPECIALIZATION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Thresholds<double> specialization.
 * \ingroup core
 */
template<>
struct Thresholds<double>
{
public:
   //! Threshold for the contact classification.
   /*! This threshold separates between separating, resting and colliding contacts. */
   static inline double collisionThreshold() { return 1E-8; }

   //! Threshold for the distance between two rigid bodies.
   /*! Rigid bodies with a distance smaller than this threshold are in contact. */
   static inline double contactThreshold() { return 1E-8; }

   //! Threshold for the restriction of the coefficient of restitution.
   /*! In case the relative velocity between two colliding rigid bodies is smaller than this
       threshold, a coefficient of restitution of 0 is used to avoid an infinite number of
       collisions during a single time step. */
   static inline double restitutionThreshold() { return 1E-8; }

   //! Threshold for the separation between static and dynamic friction.
   /*! This threshold represents the boundary between static and dynamic friction. */
   static inline double frictionThreshold() { return 1E-8; }

   //! Threshold for surface points/checks.
   /*! Only points with a distance to the surface smaller than this threshold are considered
       surface point. */
   static inline double surfaceThreshold() { return 5E-7; }

   //! Threshold for parallelism checks.
   /*! Scalar products smaller than this threshold value indicate parallel vectors. */
   static inline double parallelThreshold() { return 1E-8; }
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  LONG DOUBLE SPECIALIZATION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Thresholds<long double> specialization.
 * \ingroup core
 */
template<>
struct Thresholds<long double>
{
public:
   //! Threshold for the contact classification.
   /*! This threshold separates between separating, resting and colliding contacts. */
   static inline long double collisionThreshold() { return 1E-10L; }

   //! Threshold for the distance between two rigid bodies.
   /*! Rigid bodies with a distance smaller than this threshold are in contact. */
   static inline long double contactThreshold() { return 5E-7L; }

   //! Threshold for the restriction of the coefficient of restitution.
   /*! In case the relative velocity between two colliding rigid bodies is smaller than this
       threshold, a coefficient of restitution of 0 is used to avoid an infinite number of
       collisions during a single time step. */
   static inline long double restitutionThreshold() { return 1E-8L; }

   //! Threshold for the separation between static and dynamic friction.
   /*! This threshold represents the boundary between static and dynamic friction. */
   static inline long double frictionThreshold() { return 1E-8L; }

   //! Threshold for surface points/checks.
   /*! Only points with a distance to the surface smaller than this threshold are considered
       surface point. */
   static inline long double surfaceThreshold() { return 5E-7L; }

   //! Threshold for parallelism checks.
   /*! Scalar products smaller than this threshold value indicate parallel vectors. */
   static inline long double parallelThreshold() { return 1E-8L; }
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL THRESHOLD VALUES
//
//=================================================================================================

// Disable false warnings in GCC 5
#if ( defined __GNUC__ ) && ( __GNUC__ >= 5 )
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wunused-variable"
#endif

//*************************************************************************************************
//! Threshold for the contact classification.
/*! This threshold separates between separating, resting and colliding contacts. */
const real collisionThreshold = Thresholds<real>::collisionThreshold();
//*************************************************************************************************


//*************************************************************************************************
//! Threshold for the distance between two rigid bodies.
/*! Rigid bodies with a distance smaller than this threshold are in contact. */
const real contactThreshold = Thresholds<real>::contactThreshold();
//*************************************************************************************************


//*************************************************************************************************
//! Threshold for the restriction of the coefficient of restitution.
/*! In case the relative velocity between two colliding rigid bodies is smaller than this
    threshold, a coefficient of restitution of 0 is used to avoid an infinite number of
    collisions during a single time step. */
const real restitutionThreshold = Thresholds<real>::restitutionThreshold();
//*************************************************************************************************


//*************************************************************************************************
//! Threshold for the separation between static and dynamic friction.
/*! This threshold represents the boundary between static and dynamic friction. */
const real frictionThreshold = Thresholds<real>::frictionThreshold();
//*************************************************************************************************


//*************************************************************************************************
//! Threshold for surface points/checks.
/*! Only points with a distance to the surface smaller than this threshold are considered
    surface point. */
const real surfaceThreshold = Thresholds<real>::surfaceThreshold();
//*************************************************************************************************


//*************************************************************************************************
//! Threshold for parallelism checks.
/*! Scalar products smaller than this threshold value indicate parallel vectors. */
const real parallelThreshold = Thresholds<real>::parallelThreshold();
//*************************************************************************************************

} // namespace pe

#endif
