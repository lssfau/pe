//=================================================================================================
/*!
 *  \file pe/config/DEMConfig.h
 *  \brief Configuration file for the discrete element solver
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
/*!\brief Selection of the normal force model for the discrete element solver.
 * \ingroup config
 *
 * The \a forceModel setting switches between different models for the evaluation of the normal
 * force within the discrete element solver. For a detailed explanation of the different force
 * models see the ForceModel class description. Currently, the following models are available:
 *   - basic
 *   - hertz
 */
const ForceModel forceModel = hertz;
//*************************************************************************************************
