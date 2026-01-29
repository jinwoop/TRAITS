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
#include "traits/geometric_planning/configurations/graph_configuration.hpp"

// External
#include <fmt/format.h>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"

namespace traits
{
    GraphConfiguration::GraphConfiguration(GraphType graph_type, unsigned int id)
            : ConfigurationBase(ConfigurationType::e_graph)
            , m_graph_type(graph_type)
            , m_id(id)
    {}
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<const traits::GraphConfiguration>
    adl_serializer<std::shared_ptr<const traits::GraphConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<traits::GraphConfiguration>>::from_json(j);
    }

    std::shared_ptr<traits::GraphConfiguration>
    adl_serializer<std::shared_ptr<traits::GraphConfiguration>>::from_json(const json& j)
    {
        traits::json_ext::validateJson(j, {{traits::constants::k_graph_type, nlohmann::json::value_t::string}});
        const traits::GraphType type = j.at(traits::constants::k_graph_type);

        switch(type)
        {
            case traits::GraphType::e_euclidean:
            {
                return j.get<std::shared_ptr<traits::EuclideanGraphConfiguration>>();
            }
            case traits::GraphType::e_grid:
            {
                // TODO(Andrew)
                throw traits::createLogicError("Not implemented");
            }
            default:
            {
                throw traits::createLogicError(
                        fmt::format("Unknown GraphType: {0:s}",
                                    j.at(traits::constants::k_graph_type).get<std::string>()));
            }
        }
    }
    void adl_serializer<std::shared_ptr<const traits::GraphConfiguration>>::to_json(
            json& j,
            const std::shared_ptr<const traits::GraphConfiguration>& c)
    {
        switch(c->graphType())
        {
            case traits::GraphType::e_euclidean:
            {
                j = std::dynamic_pointer_cast<const traits::EuclideanGraphConfiguration>(c);
                return;
            }
            case traits::GraphType::e_grid:
            {
                // TODO(Andrew)
                throw traits::createLogicError("Not implemented");
            }
            default:
            {
                throw traits::createLogicError("Unknown GraphType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<traits::GraphConfiguration>>::to_json(
            json& j,
            const std::shared_ptr<traits::GraphConfiguration>& c)
    {
        adl_serializer<std::shared_ptr<const traits::GraphConfiguration>>::to_json(j, c);
    }
}  // namespace nlohmann
