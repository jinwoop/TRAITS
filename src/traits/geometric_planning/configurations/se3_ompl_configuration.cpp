/* Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020–2023
 *
 * Author: Andrew Messing
 * Author: Glen Neville
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
#include "traits/geometric_planning/configurations/se3_ompl_configuration.hpp"

// External
#include <fmt/format.h>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/configurations/se3_state_ompl_configuration.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"

namespace traits
{
    Se3OmplConfiguration::Se3OmplConfiguration(OmplGoalType goal_type)
            : OmplConfiguration(goal_type, OmplStateSpaceType::e_se3)
    {}
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<const traits::Se3OmplConfiguration>
    adl_serializer<std::shared_ptr<const traits::Se3OmplConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<traits::Se3OmplConfiguration>>::from_json(j);
    }

    std::shared_ptr<traits::Se3OmplConfiguration>
    adl_serializer<std::shared_ptr<traits::Se3OmplConfiguration>>::from_json(const json& j)
    {
        traits::json_ext::validateJson(j, {{traits::constants::k_goal_type, nlohmann::json::value_t::string}});
        const traits::OmplGoalType type = j.at(traits::constants::k_goal_type);
        switch(type)
        {
            case traits::OmplGoalType::e_state:
            {
                return j.get<std::shared_ptr<traits::Se3StateOmplConfiguration>>();
            }
            case traits::OmplGoalType::e_set_of_states:
            {
                throw traits::createLogicError("Not implemented");
            }
            case traits::OmplGoalType::e_space:
            {
                throw traits::createLogicError("Not implemented");
            }
            default:
            {
                throw traits::createLogicError(
                        fmt::format("Unknown OmplGoalType: {0:s}",
                                    j.at(traits::constants::k_goal_type).get<std::string>()));
            }
        }
    }
    void adl_serializer<std::shared_ptr<const traits::Se3OmplConfiguration>>::to_json(
            json& j,
            const std::shared_ptr<const traits::Se3OmplConfiguration>& c)
    {
        switch(c->goalType())
        {
            case traits::OmplGoalType::e_state:
            {
                j = std::dynamic_pointer_cast<const traits::Se3StateOmplConfiguration>(c);
                return;
            }
            case traits::OmplGoalType::e_set_of_states:
            {
                throw traits::createLogicError("Not implemented");
            }
            case traits::OmplGoalType::e_space:
            {
                throw traits::createLogicError("Not implemented");
            }
            default:
            {
                throw traits::createLogicError("Unknown OmplGoalType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<traits::Se3OmplConfiguration>>::to_json(
            json& j,
            const std::shared_ptr<traits::Se3OmplConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<const traits::Se3OmplConfiguration>>::to_json(j, c);
    }
}  // namespace nlohmann
