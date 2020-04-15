//=================================================================================================
/*!
 *  \file pe/config/POVRay.h
 *  \brief Configuration file for the POV-Ray visualization module
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
/*!\brief File generation strategy for the POV-Ray visualization.
 * \ingroup config
 *
 * This flag value specifies the file generation strategy for the POV-Ray visualization during
 * MPI parallel simulations. In case the value is set to \a true, a single file is generated
 * for every specified time step. For MPI parallel simulations this means that the information
 * from all processes has to be merged into a single file. Although a single file for every
 * time step is nice to handle, this strategy does not scale well with a large number of MPI
 * processes and is therefore NOT recommended for large-scale simulations. In case the value
 * is set to \a false, the visualization data is spread across multiple files (one for each
 * process). Although this strategy is less convenient, it scales perfectly with an arbitrary
 * number of MPI processes and is therefore recommended for large-scale parallel simulations.
 */
const bool singleFile = false;
//*************************************************************************************************
