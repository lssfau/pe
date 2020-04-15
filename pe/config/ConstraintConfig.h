//=================================================================================================
/*!
 *  \file pe/config/ConstraintConfig.h
 *  \brief Configuration file for the motion constraints
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
/*! \brief Switch for the coefficient of restitution of contacts.
 * \ingroup config
 *
 * This value controls the restitution of contacts. If it is set to \b false, a zero
 * coefficient of restitution will be used. If it is set to \b true, the corresponding
 * coefficient of restitution will be considered for the simulation of contacts.
 */
const bool bouncing = true;
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Threshold for the error reduction.
 * \ingroup config
 *
 * This value specifies the threshold for the error reduction of joints. If the value of the
 * correction parameter of a joint (\f$ k_{erp} k_{fps} \f$) is less than this threshold, the
 * error correction will not try to correct the errors any more. For a small threshold value
 * (e.g. \f$ 1e-9 \f$), the solution will be more accurate, whereas a large threshold value
 * (e.g. \f$ 1e-5 \f$) will be considerably faster.
 */
const real erpthreshold = static_cast<real>( 5e-7 );
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Error correction parameter for contacts.
 * \ingroup config
 *
 * This value controls the error correction applied to contacts between rigid bodies
 * (\f$ k_{erp_{contact}} \f$). It has to be in the range \f$ 0 \leq k_{erp_{contact}} \leq 1 \f$.
 * Note that setting \f$ k_{erp_{contact}} = 1 \f$ does not perfectly correct all errors due
 * to various internal approximations. It is recommended to use a value betweeen 0.1 and 0.8.
 */
const real errCorContact = static_cast<real>( 0.1 );
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Error correction parameter for ball joints.
 * \ingroup config
 *
 * This value controls the error correction applied to ball joints (\f$ k_{erp_{ball}} \f$).
 * It has to be in the range \f$ 0 \leq k_{erp_{ball}} \leq 1 \f$. Note that setting
 * \f$ k_{erp_{ball}} = 1 \f$ does not perfectly correct all errors due to various internal
 * approximations. It is recommended to use a value betweeen 0.1 and 0.8.
 */
const real errCorBall = static_cast<real>( 0.5 );
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Error correction parameter for fixed joints.
 * \ingroup config
 *
 * This value controls the error correction applied to fixed joints (\f$ k_{erp_{fixed}} \f$).
 * It can be in the range \f$ 0 \leq k_{erp_{fixed}} \leq 1 \f$. Note that setting
 * \f$ k_{erp_{fixed}} = 1 \f$ does not perfectly correct all errors due to various internal
 * approximations. It is recommended to use a value betweeen 0.1 and 0.8.
 */
const real errCorFixed = static_cast<real>( 0.2 );
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Error correction parameter for hinge joints.
 * \ingroup config
 *
 * This value controls the error correction applied to hinge joints (\f$ k_{erp_{hinge}} \f$).
 * It can be in the range \f$ 0 \leq k_{erp_{hinge}} \leq 1 \f$. Note that setting
 * \f$ k_{erp_{hinge}} = 1 \f$ does not correct all errors entirely due to various internal
 * approximations. It is recommended to use a value betweeen 0.1 and 0.8.
 */
const real errCorHinge = static_cast<real>( 0.3 );
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Error correction parameter for slider joints.
 * \ingroup config
 *
 * This value controls the error correction applied to slider joints (\f$ k_{erp_{slider}} \f$).
 * It can be in the range \f$ 0 \leq k_{erp_{slider}} \leq 1 \f$. Note that setting
 * \f$ k_{erp_{slider}} = 1 \f$ does not correct all errors entirely due to various internal
 * approximations. It is recommended to use a value betweeen 0.1 and 0.8.
 */
const real errCorSlider = static_cast<real>( 0.3 );
//*************************************************************************************************

