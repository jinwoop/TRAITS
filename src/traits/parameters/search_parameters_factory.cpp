/* Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Andrew Messing
 * Author: Glen Neville
 * Author: Jinwoo Park
 *
 * Copyright (C) 2020–2023 Andrew Messing
 * Copyright (C) 2020–2023 Glen Neville
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
#include "traits/parameters/search_parameters_factory.hpp"

// Local
#include "traits/common/utilities/constants.hpp"

namespace traits
{
    SearchParametersFactory& SearchParametersFactory::instance()
    {
        static SearchParametersFactory singleton;
        return singleton;
    }
    SearchParametersFactory::SearchParametersFactory()
            : AlgorithmParametersFactoryBase(constants::k_search_parameters)
    {
        // Set parent tree
        setParent(constants::k_best_first_search_parameters, constants::k_search_parameters);
        setParent(constants::k_focal_a_star_parameters, constants::k_best_first_search_parameters);

        // Set required parameters
        setRequired(constants::k_search_parameters,
                    {{constants::k_has_timeout, nlohmann::json::value_t::boolean},
                     {constants::k_timeout, nlohmann::json::value_t::number_float},
                     {constants::k_timer_name, nlohmann::json::value_t::string}});
        setRequired(constants::k_best_first_search_parameters, {});
        setRequired(constants::k_focal_a_star_parameters,
                    {{constants::k_w, nlohmann::json::value_t::number_float},
                     {constants::k_rebuild, nlohmann::json::value_t::boolean}});

        // Set optional parameters
        setOptional(constants::k_search_parameters, {});
        setOptional(constants::k_best_first_search_parameters,
                    {{constants::k_save_pruned_nodes, nlohmann::json::value_t::boolean},
                     {constants::k_save_closed_nodes, nlohmann::json::value_t::boolean},
                     {constants::k_alpha, nlohmann::json::value_t::number_float}}),
        setOptional(constants::k_focal_a_star_parameters, {});

        // Set default values for optional parameters
        setDefault(constants::k_search_parameters, {});
        setDefault(constants::k_best_first_search_parameters,
                   {{constants::k_save_pruned_nodes, false},
                    {constants::k_save_closed_nodes, false},
                    {constants::k_alpha, 0.5}}),
        setDefault(constants::k_focal_a_star_parameters, {});
    }
}  // namespace traits
