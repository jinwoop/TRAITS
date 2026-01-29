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
#include "traits/geometric_planning/configurations/configuration_base.hpp"

// External
#include <fmt/format.h>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/configurations/graph_configuration.hpp"
#include "traits/geometric_planning/configurations/ompl_configuration.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"

namespace traits
{
    ConfigurationBase::ConfigurationBase(ConfigurationType type)
            : m_configuration_type(type)
    {}
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<const traits::ConfigurationBase>
    adl_serializer<std::shared_ptr<const traits::ConfigurationBase>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<traits::ConfigurationBase>>::from_json(j);
    }

    std::shared_ptr<traits::ConfigurationBase>
    adl_serializer<std::shared_ptr<traits::ConfigurationBase>>::from_json(const json& j)
    {
        traits::json_ext::validateJson(
                j,
                {{traits::constants::k_configuration_type, nlohmann::json::value_t::string}});

        const traits::ConfigurationType configuration_type = j.at(traits::constants::k_configuration_type);
        switch(configuration_type)
        {
            case traits::ConfigurationType::e_ompl:
            {
                return j.get<std::shared_ptr<traits::OmplConfiguration>>();
            }
            case traits::ConfigurationType::e_graph:
            {
                return j.get<std::shared_ptr<traits::GraphConfiguration>>();
            }
            default:
            {
                throw traits::createLogicError(
                        fmt::format("Unknown ConfigurationType: {0:s}",
                                    j.at(traits::constants::k_configuration_type).get<std::string>()));
            }
        }
    }

    void adl_serializer<std::shared_ptr<const traits::ConfigurationBase>>::to_json(
            json& j,
            const std::shared_ptr<const traits::ConfigurationBase>& c)
    {
        switch(c->configurationType())
        {
            case traits::ConfigurationType::e_ompl:
            {
                j = std::dynamic_pointer_cast<const traits::OmplConfiguration>(c);
                return;
            }
            case traits::ConfigurationType::e_graph:
            {
                j = std::dynamic_pointer_cast<const traits::GraphConfiguration>(c);
                return;
            }
            default:
            {
                throw traits::createLogicError("Unknown ConfigurationType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<traits::ConfigurationBase>>::to_json(
            json& j,
            const std::shared_ptr<traits::ConfigurationBase>& c)
    {
        adl_serializer<std::shared_ptr<const traits::ConfigurationBase>>::to_json(j, c);
    }
}  // namespace nlohmann
