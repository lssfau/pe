//=================================================================================================
/*!
 *  \file pe/core/batches/typetraits/IsSameGenerator.h
 *  \brief Header file for the IsSameGenerator type trait
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

#ifndef _PE_CORE_BATCHES_TYPETRAITS_ISSAMEGENERATOR_H_
#define _PE_CORE_BATCHES_TYPETRAITS_ISSAMEGENERATOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check for batch generator types.
 * \ingroup batch_generation_type_traits
 *
 * This class tests if the two data types \a A and \a B are the same batch generator types.
 * If \a A and \a B are the same data type, then the \a value member enumeration is set to 1,
 * the nested type definition \a Type is \a TrueType, and the class derives from \a TrueType.
 * Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives from
 * \a FalseType.

   \code
   pe::IsSameGenerator< SingleBatch, SingleBatch >::value  // Evaluates to 1
   pe::IsSameGenerator< UnionFind, UnionFind >::Type       // Results in TrueType
   pe::IsSameGenerator< SingleBatch, SingleBatch >         // Is derived from TrueType
   pe::IsSameGenerator< SingleBatch, UnionFind >::value    // Evaluates to 0
   pe::IsSameGenerator< UnionFind, SingleBatch >::Type     // Results in FalseType
   pe::IsSameGenerator< SingleBatch, UnionFind >           // Is derived from FalseType
   \endcode
 */
template< template<typename> class A
        , template<typename> class B >
struct IsSameGenerator : public FalseType
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = 0 };
   typedef FalseType  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsSameGenerator type trait for two equal batch generator types.
template< template<typename> class T >
struct IsSameGenerator<T,T> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace batches

} // namespace pe

#endif
