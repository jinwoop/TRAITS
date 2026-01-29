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
#include "traits/species.hpp"

// External
#include <magic_enum/magic_enum.hpp>
// Local
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"

namespace traits
{
    unsigned int Species::s_num_species = 0;
    unsigned int Species::s_next_id     = 0;

    Species::Species()
            : m_id(s_next_id++)
    {
        ++s_num_species;
    }

    Species::Species(const std::string& name,
                             const Eigen::VectorXf& traits_max,
                             const Eigen::VectorXf& trait_rates_max,
                             const Eigen::VectorXd& traits_provisionable,
                             const Eigen::VectorXd& traits_exhaustible,
                             const Eigen::VectorXf& current_trait_functions_coeff,
                             const Eigen::VectorXf& current_trait_rate_functions_coeff,
                             const double max_battery_capacity,
                             const float peukert_coeff,
                             const float idle_current,
                             const float max_possible_current,
                             const float radius,
                             const float speed,
                             const float speed_coeff,
                             const std::shared_ptr<TraitsOmplMotionPlanner>& motion_planner)
            : m_id(s_next_id++)
            , m_name(name)
            , m_traits_max(traits_max)
            , m_trait_rates_max(trait_rates_max)
            , m_traits_provisionable(traits_provisionable)
            , m_traits_exhaustible(traits_exhaustible)
            , m_current_trait_functions_coeff(current_trait_functions_coeff)
            , m_current_trait_rate_functions_coeff(current_trait_rate_functions_coeff)
            , m_max_battery_capacity(max_battery_capacity)
            , m_max_possible_current(max_possible_current)
            , m_peukert_coeff(peukert_coeff)
            , m_idle_current(idle_current)
            , m_bounding_radius(radius)
            , m_speed(speed)
            , m_speed_coeff(speed_coeff)
            , m_motion_planner(motion_planner)
    {
        ++s_num_species;
    }

    Species::~Species()
    {
        // When all species are destructed reset the next id
        --s_num_species;
        if(s_num_species == 0)
        {
            s_next_id = 0;
        }
    }

    std::shared_ptr<const Species> Species::loadJson(
            const nlohmann::json& j,
            const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& motion_planners)
    {
        json_ext::validateJson(j,
                               {{constants::k_name, nlohmann::json::value_t::string},
                                {constants::k_traits_max, nlohmann::json::value_t::array},
                                {constants::k_trait_rates_max, nlohmann::json::value_t::array},
                                {constants::k_traits_provisionable, nlohmann::json::value_t::array},
                                {constants::k_traits_exhaustible, nlohmann::json::value_t::array},
                                {constants::k_current_trait_functions_coeff, nlohmann::json::value_t::array},
                                {constants::k_current_trait_rate_functions_coeff, nlohmann::json::value_t::array},
                                {constants::k_idle_current, nlohmann::json::value_t::number_float},
                                {constants::k_peukert_coeff, nlohmann::json::value_t::number_float},
                                {constants::k_max_battery_capacity, nlohmann::json::value_t::number_float},
                                {constants::k_max_possible_current, nlohmann::json::value_t::number_float},
                                {constants::k_bounding_radius, nlohmann::json::value_t::number_float},
                                {constants::k_speed, nlohmann::json::value_t::number_float},
                                {constants::k_speed_coeff, nlohmann::json::value_t::number_float},
                                {constants::k_mp_index, nlohmann::json::value_t::number_unsigned}});

        std::shared_ptr<Species> s = std::make_shared<Species>();

        j.at(constants::k_name).get_to<std::string>(s->m_name);
        j.at(constants::k_traits_max).get_to<Eigen::VectorXf>(s->m_traits_max);
        j.at(constants::k_trait_rates_max).get_to<Eigen::VectorXf>(s->m_trait_rates_max);
        j.at(constants::k_traits_provisionable).get_to<Eigen::VectorXd>(s->m_traits_provisionable);
        j.at(constants::k_traits_exhaustible).get_to<Eigen::VectorXd>(s->m_traits_exhaustible);
        j.at(constants::k_idle_current).get_to<float>(s->m_idle_current);
        j.at(constants::k_current_trait_functions_coeff).get_to<Eigen::VectorXf>(s->m_current_trait_functions_coeff);
        j.at(constants::k_current_trait_rate_functions_coeff).get_to<Eigen::VectorXf>(s->m_current_trait_rate_functions_coeff);
        j.at(constants::k_max_battery_capacity).get_to<double>(s->m_max_battery_capacity);
        j.at(constants::k_max_possible_current).get_to<float>(s->m_max_possible_current);
        j.at(constants::k_peukert_coeff).get_to<float>(s->m_peukert_coeff);
        j.at(constants::k_bounding_radius).get_to<float>(s->m_bounding_radius);
        j.at(constants::k_speed).get_to<float>(s->m_speed);
        j.at(constants::k_speed_coeff).get_to<float>(s->m_speed_coeff);

        unsigned int mp_index;
        j.at(constants::k_mp_index).get_to<unsigned int>(mp_index);
        assert(mp_index < motion_planners.size());
        s->m_motion_planner = motion_planners[mp_index];

        return s;
    }
}  // namespace traits
