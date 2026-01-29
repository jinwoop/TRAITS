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
#include "traits/geometric_planning/configurations/euclidean_graph_configuration.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"

namespace traits
{
    EuclideanGraphConfiguration::EuclideanGraphConfiguration(unsigned int id, float x, float y)
            : GraphConfiguration(GraphType::e_euclidean, id)
            , m_x(x)
            , m_y(y)
    {}

    float EuclideanGraphConfiguration::euclideanDistance(const ConfigurationBase& rhs) const
    {
        if(m_configuration_type != rhs.configurationType())
        {
            throw createLogicError("Cannot compute the euclidean distance for two configurations of different types");
        }

        const auto* rhs_graph = dynamic_cast<const GraphConfiguration*>(&rhs);
        if(!rhs_graph)
        {
            throw createLogicError("'rhs' claims it is a GraphConfiguration, but it is not");
        }
        if(m_graph_type != rhs_graph->graphType())
        {
            throw createLogicError(
                    "Cannot compute the euclidean distance for two graph configurations of different types");
        }

        const auto* rhs_point_graph = dynamic_cast<const EuclideanGraphConfiguration*>(rhs_graph);
        if(!rhs_point_graph)
        {
            throw createLogicError("'rhs' claims it is a PointGraphConfiguration, but it is not");
        }

        return euclideanDistance(*rhs_point_graph);
    }

    float EuclideanGraphConfiguration::euclideanDistance(const EuclideanGraphConfiguration& rhs) const
    {
        // Note: split up because of some bug with sqrt
        const float x_diff    = static_cast<float>(m_x) - static_cast<float>(rhs.m_x);
        const float y_diff    = static_cast<float>(m_y) - static_cast<float>(rhs.m_y);
        const float x_diff_sq = powf(x_diff, 2.0f);
        const float y_diff_sq = powf(y_diff, 2.0f);
        const float sq_sum    = x_diff_sq + y_diff_sq;
        return std::sqrt(sq_sum);
    }

    bool EuclideanGraphConfiguration::operator==(const ConfigurationBase& rhs) const
    {
        if(m_configuration_type != rhs.configurationType())
        {
            return false;
        }

        const auto* rhs_graph = dynamic_cast<const GraphConfiguration*>(&rhs);
        if(!rhs_graph || m_graph_type != rhs_graph->graphType())
        {
            return false;
        }

        const auto* rhs_point_graph = dynamic_cast<const EuclideanGraphConfiguration*>(&rhs);
        if(!rhs_point_graph)
        {
            return false;
        }

        return *this == *rhs_point_graph;
    }

    bool EuclideanGraphConfiguration::operator==(const EuclideanGraphConfiguration& rhs) const
    {
        return m_id == rhs.m_id && m_x == rhs.m_x && m_y == rhs.m_y;
    }

    void to_json(nlohmann::json& j, const EuclideanGraphConfiguration& c)
    {
        j[traits::constants::k_configuration_type] = c.configurationType();
        j[traits::constants::k_graph_type]         = c.graphType();
        j[traits::constants::k_id]                 = c.id();
        j[traits::constants::k_x]                  = c.x();
        j[traits::constants::k_y]                  = c.y();
    }
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<const traits::EuclideanGraphConfiguration>
    adl_serializer<std::shared_ptr<const traits::EuclideanGraphConfiguration>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<traits::EuclideanGraphConfiguration>>::from_json(j);
    }

    std::shared_ptr<traits::EuclideanGraphConfiguration>
            adl_serializer<std::shared_ptr<traits::EuclideanGraphConfiguration>>::from_json(const json& j)
{
    traits::json_ext::validateJson(j,
{{traits::constants::k_id, nlohmann::json::value_t::number_unsigned},
{traits::constants::k_x, nlohmann::json::value_t::number_float},
{traits::constants::k_y, nlohmann::json::value_t::number_float}});
const unsigned int id = j[traits::constants::k_id];
const float x         = j[traits::constants::k_x];
const float y         = j[traits::constants::k_y];
return std::make_shared<traits::EuclideanGraphConfiguration>(id, x, y);
}

void adl_serializer<std::shared_ptr<const traits::EuclideanGraphConfiguration>>::to_json(
        json& j,
        const std::shared_ptr<const traits::EuclideanGraphConfiguration>& c)
{
    j = *c;
}

void adl_serializer<std::shared_ptr<traits::EuclideanGraphConfiguration>>::to_json(
        json& j,
const std::shared_ptr<traits::EuclideanGraphConfiguration>& c)
{
adl_serializer<std::shared_ptr<const traits::EuclideanGraphConfiguration>>::to_json(j, c);
}
}  // namespace nlohmann
