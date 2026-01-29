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
#include "traits/parameters/trait_distribution_parameters_factory.hpp"

// Local
#include "traits/common/utilities/constants.hpp"

namespace traits
{
    TraitDistributionParametersFactory& TraitDistributionParametersFactory::instance()
    {
        static TraitDistributionParametersFactory singleton;
        return singleton;
    }

    TraitDistributionParametersFactory::TraitDistributionParametersFactory()
            : AlgorithmParametersFactoryBase(constants::k_trait_distribution_parameters)
    {

        // Set required parameters
        setRequired(constants::k_trait_distribution_parameters,
                    {{constants::k_timeout, nlohmann::json::value_t::number_float},
                     {constants::k_nlp_timeout, nlohmann::json::value_t::number_float},
                    });

        // Set optional parameters
        setOptional(constants::k_trait_distribution_parameters,
                    {{constants::k_threads, nlohmann::json::value_t::number_unsigned}});

        // Set default values for optional parameters
        setDefault(constants::k_trait_distribution_parameters,
                   {{constants::k_threads, 0}});
    }
}  // namespace traits
