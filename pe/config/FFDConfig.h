//=================================================================================================
/*!
 *  \file pe/config/FFDConfig.h
 *  \brief Configuration file for the fast frictional dynamics solver
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
/*!\brief Threshold value for the characterization of contacts.
 * \ingroup config
 *
 * This threshold value specifies when a contact is considered separating and is not treated by
 * the fast frictional dynamics algorithm. All contacts with a constraint violation above this
 * threshold value are rejected, all contacts with a constraint violation below this value are
 * treated during the collision resolution. The default value is 0.
 */
const real contactTolerance = static_cast<real>( 0 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Restriction on the maximum possible collision response \f$ [0..\infty) \f$.
 * \ingroup config
 *
 * This value offers the option to restrict the maximum possible collision response of the fast
 * frictional dynamics (FFD) algorithm. If the collision responses are not bounded (i.e. if this
 * value is set to 0), the collision response (that is directly translated to the velocity of
 * the rigid bodies) may be arbitrarily high. In certain situations (as for instance in case of
 * two nearly parallel collision constraints on a specific rigid body) this may result in extreme
 * velocities that may (and surely will) break the simulation. This value allows to restrict the
 * maximum collision response to feasible values. The smaller the setting of this value, the
 * lower is the expected collision response. Note however that this may cause some overlaps
 * between rigid bodies!
 *
 * The \a responseRestriction value must be in the range \f$ [0..\infty) \f$. The default value
 * for this setting is 1.2.
 */
const real responseRestriction = static_cast<real>( 1.2 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Penetration correction factor \f$ [0..1] \f$
 * \ingroup config
 *
 * The penetration correction factor specifies the intensity of the attempt of the fast frictional
 * dynamics (FFD) algorithm to correct interpenetrations of colliding rigid bodies. Depending on
 * the magnitude of the overlap the solver applies additional force to the two colliding bodies
 * to resolve the interpenetration between the two bodies. Note that this force is an artificial
 * force to compensate for the inevitable interpenetrations due to discrete time stepping. Hence
 * this force results in a certain amount of artificial energy added to the simulation system.
 * Consequently, the penetration correction factor should be chosen such that both penetrations
 * and the artificial energy gain are minimized.
 *
 * The \a penetrationCorrection value must be in the range \f$ [0..1] \f$. The default value for
 * this value is 0.1.
 */
const real penetrationCorrection = static_cast<real>( 0.1 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Friction flag for the contact treatment of the fast frictional dynamics solver.
 * \ingroup config
 *
 * This flag value specifies whether frictional collision responses are calculated by the FFD
 * solver or not. A value of \a true activates the calculation of a frictional responses, a
 * value of \a false deactivates any friction calculations.\n
 * Turning the friction calculations off slightly increases the performance of the FFD solver
 * solver by approx. a factor of 2. However, frictionless collisions may considerably diminish
 * the visual credibility of the calculations.
 *
 * Possible settings for the friction flag: \a true / \a false.
 */
const bool friction = true;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Number of friction samples per contact point.
 * \ingroup config
 *
 * This value sets the number of friction samples per contact point. Smaller value increase the
 * performance of the simulation, whereas larger values will result in more accurate frictional
 * responses.
 *
 * The number of friction samples must be an integral value in the range \f$ [1..\infty) \f$.
 * The default number of samples is 4.
 */
const size_t frictionSamples = 4;
//*************************************************************************************************
