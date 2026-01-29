/* Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Jinwoo Park
 *
 * Copyright (C) 2026 Jinwoo Park
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

// region Includes
// Local
#include "traits/parameters/algorithm_parameters_factory_base.hpp"
// endregion

namespace traits
{
    /*!
     * \class TraitDistributionParametersFactory
     * \brief Factory to build parameters for a trait distribution algorithm
     */
    class TraitDistributionParametersFactory : public AlgorithmParametersFactoryBase
    {
    public:
        static TraitDistributionParametersFactory& instance();

        // region Special Member Functions
    private:
        //! Default Constructor
        TraitDistributionParametersFactory();

    public:
        //! Copy Constructor
        TraitDistributionParametersFactory(const TraitDistributionParametersFactory&) = delete;
        //! Move Constructor
        TraitDistributionParametersFactory(TraitDistributionParametersFactory&&) = delete;
        //! Destructor
        ~TraitDistributionParametersFactory() = default;
        //! Copy Assignment Operator
        TraitDistributionParametersFactory& operator=(const TraitDistributionParametersFactory&) = delete;
        //! Move Assignment Operator
        TraitDistributionParametersFactory& operator=(TraitDistributionParametersFactory&&) = delete;
        // endregion
    };  // class TraitDistributionParametersFactory
}  // namespace traits
