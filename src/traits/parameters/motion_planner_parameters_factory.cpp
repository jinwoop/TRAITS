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
#include "traits/parameters/motion_planner_parameters_factory.hpp"

// Local
#include "traits/common/utilities/constants.hpp"

namespace traits
{
    MotionPlannerParametersFactory& MotionPlannerParametersFactory::instance()
    {
        static MotionPlannerParametersFactory singleton;
        return singleton;
    }

    MotionPlannerParametersFactory::MotionPlannerParametersFactory()
            : AlgorithmParametersFactoryBase(constants::k_motion_planner_parameters)
    {
        setParent(constants::k_euclidean_graph_motion_planner_parameters, constants::k_motion_planner_parameters);
        setParent(constants::k_ompl_motion_planner_parameters, constants::k_motion_planner_parameters);

        setRequired(constants::k_motion_planner_parameters,
                    {{constants::k_timeout, nlohmann::json::value_t::number_float}});
        setRequired(constants::k_ompl_motion_planner_parameters,
                    {{traits::constants::k_timeout, nlohmann::json::value_t::number_float},
                     {traits::constants::k_simplify_path, nlohmann::json::value_t::boolean},
                     {traits::constants::k_simplify_path_timeout, nlohmann::json::value_t::number_float},
                     {traits::constants::k_ompl_mp_algorithm, nlohmann::json::value_t::string}});
        setRequired(constants::k_euclidean_graph_motion_planner_parameters,
                    {{constants::k_is_complete, nlohmann::json::value_t::boolean}});

        setOptional(constants::k_motion_planner_parameters, {});
        setOptional(constants::k_ompl_motion_planner_parameters,
                    {{traits::constants::k_solutions_window, nlohmann::json::value_t::number_unsigned},
                     {traits::constants::k_convergence_epsilon, nlohmann::json::value_t::number_float}});
        setOptional(constants::k_euclidean_graph_motion_planner_parameters, {});

        setDefault(constants::k_motion_planner_parameters, {});
        setDefault(constants::k_ompl_motion_planner_parameters,
                   {{traits::constants::k_solutions_window, 10}, {traits::constants::k_convergence_epsilon, 0.1f}});
        setDefault(constants::k_euclidean_graph_motion_planner_parameters, {});
    }
}  // namespace traits
