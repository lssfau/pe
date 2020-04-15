//=================================================================================================
/*!
 *  \file pe/core/response/ForceModel.h
 *  \brief Header file for the ForceModel
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

#ifndef _PE_CORE_RESPONSE_FORCEMODEL_H_
#define _PE_CORE_RESPONSE_FORCEMODEL_H_


namespace pe {

namespace response {

namespace dem {

//=================================================================================================
//
//  DISCRETE ELEMENT METHOD NORMAL FORCE MODELS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Type codes of the DEM normal force models.
 * \ingroup collision_response
 *
 * The DEM force models represent different approaches to evaluate the normal force occuring
 * during the collision of two rigid bodies. Currently, two different models are integrated
 * into the \b pe framework. The first model is based on a common linear force model:

              \f[ F^n = F^n_{el} + F^n_{diss} = k^n \xi + \gamma^n \dot{\xi} \f]

 * The model can be understood as a spring-dashpot system, where \f$ k^n \f$ is the stiffness of a
 * linear spring (configured via the dem::stiffnessN configuration parameter), and \f$ \gamma^n \f$
 * is a constant for a velocity dependent damper (configured via the dem::dampingN configuration
 * parameter). This model has been shown to have an analytic solution that leads to a constant
 * coefficient of restitution

   \f[ \epsilon^n = exp ( -\frac{\gamma^n}{2m_{eff}} \pi ( \frac{k^n}{m_{eff}} - ( \frac{\gamma^n}{2m_{eff}}^2 ) )^{-1/2} ) \f]

 * and constant duration of collision

   \f[ t^n = \pi ( \frac{k^n}{m_{eff}} - (\frac{\gamma^n}{2m_{eff}})^2 )^{-1/2}, \f]

 * independent of the initial velocity \f$ v_0 \f$ at the beginning of the collision. The parameter

   \f[ m_{eff} = \frac{m_a m_b}{m_a + m_b} \f]

 * is the effective mass of the colliding bodies.\\
 * The second model for the normal force is based on Heinrich Hertz's theory of the interaction
 * of elastic spheres. It consists again of a conservative and a dissipative part, but is fully
 * non-linear:

   \f[ F^n = F^n_{el} + F^n_{diss} = \hat{k}^n \xi^{3/2} + \hat{\gamma}^n \dot{\xi} \xi^{1/2}, \f]

 * with the spring stiffness \f$ \hat{k}^n \f$ and the damping constant \f$ \hat{\gamma}^n \f$.
 * The spring stiffness \f$ \hat{k}^n \f$ is dependent on the geometry and the elastic properties
 * of the spheres in contact:

   \f[ \hat{k}^n = \frac{4}{3} E_{eff} \sqrt{R_{eff}} \f]

 * with the effective radius

   \f[ R_{eff} = \frac{R_i R_j}{R_i + R_j} \f]

 * and the effective Young's modulus

   \f[ \frac{1}{E_{eff}} = \frac{1 - \nu_1^2}{E_1} + \frac{1 - \nu_2^2}{E_2}, \f]

 * The Young's modulus \f$ E \f$ and the Poisson's ratio \f$ \nu \f$ are quantities to describe
 * the stiffness and deformability of materials (see the Material class description).
 */
enum ForceModel {
   basic = 1,  //!< Code for the basic force model.
   hertz = 2   //!< Code for the Hertz force model.
};
//*************************************************************************************************

} // namespace dem

} // namespace response

} // namespace pe

#endif
