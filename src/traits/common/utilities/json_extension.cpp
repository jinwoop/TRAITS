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
#include "traits/common/utilities/json_extension.hpp"

// External
#include <magic_enum/magic_enum.hpp>
// Local
#include "traits/common/utilities/error.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"

namespace nlohmann
{
    ompl::base::RealVectorBounds adl_serializer<ompl::base::RealVectorBounds>::from_json(const json& j)
    {
        traits::json_ext::validateJson(j,
                                         {{traits::constants::k_low, nlohmann::json::value_t::array},
                                          {traits::constants::k_high, nlohmann::json::value_t::array}});

        const nlohmann::json& low  = j.at(traits::constants::k_low);
        const nlohmann::json& high = j.at(traits::constants::k_high);
        assert(low.size() == high.size());

        ompl::base::RealVectorBounds b(low.size());
        low.get_to(b.low);
        high.get_to(b.high);

        return b;
    }

    void adl_serializer<ompl::base::RealVectorBounds>::to_json(json& j, const ompl::base::RealVectorBounds& b)
    {
        j = {{traits::constants::k_low, b.low}, {traits::constants::k_high, b.high}};
    }

    void adl_serializer<ompl::base::SE2StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SE2StateSpace::StateType& s)
    {
        traits::json_ext::validateJson(j,
                                         {
                                                 {traits::constants::k_x, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_y, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_yaw, nlohmann::json::value_t::number_float},
                                         });

        // Setup state because OMPL doesn't...
        s.components    = new ompl::base::State*[2];
        s.components[0] = new ompl::base::RealVectorStateSpace::StateType();
        s.components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values = new double[2];
        s.components[1] = new ompl::base::SO2StateSpace::StateType();

        s.setX(j.at(traits::constants::k_x).get<float>());
        s.setY(j.at(traits::constants::k_y).get<float>());
        s.setYaw(j.at(traits::constants::k_yaw).get<float>());
    }

    void adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SE2StateSpace::StateType& s)
    {
        j = {{traits::constants::k_state_type, traits::OmplStateSpaceType::e_se2},
             {traits::constants::k_x, s.getX()},
             {traits::constants::k_y, s.getY()},
             {traits::constants::k_yaw, s.getYaw()}};
    }

    void adl_serializer<ompl::base::SE2StateSpace>::from_json(const json& j, ompl::base::SE2StateSpace& s)
    {
        traits::json_ext::validateJson(j, {{traits::constants::k_bounds, nlohmann::json::value_t::object}});
        s.setBounds(j.at(traits::constants::k_bounds).get<ompl::base::RealVectorBounds>());
    }

    void adl_serializer<ompl::base::SE2StateSpace>::to_json(json& j, const ompl::base::SE2StateSpace& s)
    {
        j = {{traits::constants::k_state_type, traits::OmplStateSpaceType::e_se2},
             {traits::constants::k_bounds, s.getBounds()}};
    }

    void adl_serializer<ompl::base::SE3StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SE3StateSpace::StateType& s)
    {
        traits::json_ext::validateJson(j,
                                         {
                                                 {traits::constants::k_x, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_y, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_z, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_rotation, nlohmann::json::value_t::object},
                                         });

        // Setup state because OMPL doesn't...
        s.components    = new ompl::base::State*[2];
        s.components[0] = new ompl::base::RealVectorStateSpace::StateType();
        s.components[0]->as<ompl::base::RealVectorStateSpace::StateType>()->values = new double[3];
        s.components[1] = new ompl::base::SO3StateSpace::StateType();

        s.setX(j.at(traits::constants::k_x).get<float>());
        s.setY(j.at(traits::constants::k_y).get<float>());
        s.setZ(j.at(traits::constants::k_z).get<float>());
        auto rotation =
                j.at(traits::constants::k_rotation).get<std::shared_ptr<ompl::base::SO3StateSpace::StateType>>();
        s.rotation().x = rotation->x;
        s.rotation().y = rotation->y;
        s.rotation().z = rotation->z;
        s.rotation().w = rotation->w;
    }

    void adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SE3StateSpace::StateType& s)
    {
        j = {{traits::constants::k_state_type, traits::OmplStateSpaceType::e_se3},
             {traits::constants::k_x, s.getX()},
             {traits::constants::k_y, s.getY()},
             {traits::constants::k_z, s.getZ()},
             {traits::constants::k_rotation, s.rotation()}};
    }

    void adl_serializer<ompl::base::SE3StateSpace>::from_json(const json& j, ompl::base::SE3StateSpace& s)
    {
        traits::json_ext::validateJson(j, {{traits::constants::k_bounds, nlohmann::json::value_t::object}});
        s.setBounds(j.at(traits::constants::k_bounds).get<ompl::base::RealVectorBounds>());
    }

    void adl_serializer<ompl::base::SE3StateSpace>::to_json(json& j, const ompl::base::SE3StateSpace& s)
    {
        j = {{traits::constants::k_state_type, traits::OmplStateSpaceType::e_se3},
             {traits::constants::k_bounds, s.getBounds()}};
    }

    void adl_serializer<ompl::base::SO3StateSpace::StateType>::from_json(const json& j,
                                                                         ompl::base::SO3StateSpace::StateType& s)
    {
        traits::json_ext::validateJson(j,
                                         {
                                                 {traits::constants::k_qx, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_qy, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_qz, nlohmann::json::value_t::number_float},
                                                 {traits::constants::k_qw, nlohmann::json::value_t::number_float},
                                         });

        j.at(traits::constants::k_qx).get_to(s.x);
        j.at(traits::constants::k_qy).get_to(s.y);
        j.at(traits::constants::k_qz).get_to(s.z);
        j.at(traits::constants::k_qw).get_to(s.w);
    }

    void adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(json& j,
                                                                       const ompl::base::SO3StateSpace::StateType& s)
    {
        j = {{traits::constants::k_state_type, magic_enum::enum_name(traits::OmplStateSpaceType::e_so3)},
             {traits::constants::k_qx, s.x},
             {traits::constants::k_qy, s.y},
             {traits::constants::k_qz, s.z},
             {traits::constants::k_qw, s.w}};
    }

    void adl_serializer<ompl::geometric::PathGeometric>::to_json(json& j, const ompl::geometric::PathGeometric& p)
    {
        j                            = nlohmann::json::array();
        const std::size_t num_states = p.getStateCount();
        if(num_states == 0)
        {
            return;
        }

        traits::OmplStateSpaceType state_type = traits::OmplStateSpaceType::e_unknown;
        {
            const ompl::base::State* state = p.getState(0);
            nlohmann::json j_state;
            if(const auto* se2_state = dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(j_state, *se2_state);
                state_type = traits::OmplStateSpaceType::e_se2;
            }
            else if(const auto* se3_state = dynamic_cast<const ompl::base::SE3StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(j_state, *se3_state);
                state_type = traits::OmplStateSpaceType::e_se3;
            }
            else if(const auto* so3_state = dynamic_cast<const ompl::base::SO3StateSpace::StateType*>(state))
            {
                adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(j_state, *so3_state);
                state_type = traits::OmplStateSpaceType::e_so3;
            }
            else
            {
                throw traits::createLogicError("Unknown state type");
            }
            j.push_back(j_state);
        }
        if(state_type == traits::OmplStateSpaceType::e_unknown)
        {
            throw traits::createLogicError("Unknown state type");
        }

        for(unsigned int i = 1; i < num_states; ++i)
        {
            const ompl::base::State* state = p.getState(i);
            nlohmann::json j_state;
            switch(state_type)
            {
                case traits::OmplStateSpaceType::e_se2:
                {
                    adl_serializer<ompl::base::SE2StateSpace::StateType>::to_json(
                            j_state,
                            *dynamic_cast<const ompl::base::SE2StateSpace::StateType*>(state));
                    break;
                }
                case traits::OmplStateSpaceType::e_se3:
                {
                    adl_serializer<ompl::base::SE3StateSpace::StateType>::to_json(
                            j_state,
                            *dynamic_cast<const ompl::base::SE3StateSpace::StateType*>(state));
                    break;
                }
                case traits::OmplStateSpaceType::e_so3:
                {
                    adl_serializer<ompl::base::SO3StateSpace::StateType>::to_json(
                            j_state,
                            *dynamic_cast<const ompl::base::SO3StateSpace::StateType*>(state));
                    break;
                }
            }
            j.push_back(j_state);
        }
    }
}  // namespace nlohmann

namespace traits::json_ext
{
    void validateJson(const nlohmann::json& j,
                      const std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>>& required,
                      const std::initializer_list<std::pair<const char* const, nlohmann::json::value_t>>& optionals,
                      const std::experimental::source_location location)
    {
        for(const auto& [field_name, field_type]: required)
        {
            if(!j.contains(field_name))
            {
                throw createLogicError(fmt::format("json is missing field '{0:s}'", field_name), location);
            }

            const nlohmann::json& f = j.at(field_name);
            if(f.type() != field_type)
            {
                throw createLogicError(
                        fmt::format("json field '{0:s}' should be of type '{1:s}' however is instead of type '{2:s}'",
                                    field_name,
                                    magic_enum::enum_name(field_type),
                                    magic_enum::enum_name(f.type())),
                        location);
            }
        }
        for(const auto& [field_name, field_type]: optionals)
        {
            if(!j.contains(field_name))
            {
                continue;
            }

            const nlohmann::json& f = j.at(field_name);
            if(f.type() != field_type)
            {
                throw createLogicError(
                        fmt::format("json field '{0:s}' should be of type '{1:s}' however is instead of type '{2:s}'",
                                    field_name,
                                    magic_enum::enum_name(field_type),
                                    magic_enum::enum_name(f.type())),
                        location);
            }
        }
    }

    void validateJson(const nlohmann::json& j,
                      const std::vector<std::pair<const char* const, nlohmann::json::value_t>>& required,
                      const std::vector<std::pair<const char* const, nlohmann::json::value_t>>& optionals,
                      const std::experimental::source_location location)
    {
        for(const auto& [field_name, field_type]: required)
        {
            if(!j.contains(field_name))
            {
                throw createLogicError(fmt::format("json is missing field '{0:s}'", field_name), location);
            }

            const nlohmann::json& f = j.at(field_name);
            if(f.type() != field_type)
            {
                throw createLogicError(
                        fmt::format("json field '{0:s}' should be of type '{1:s}' however is instead of type '{2:s}'",
                                    field_name,
                                    magic_enum::enum_name(field_type),
                                    magic_enum::enum_name(f.type())),
                        location);
            }
        }
        for(const auto& [field_name, field_type]: optionals)
        {
            if(!j.contains(field_name))
            {
                continue;
            }

            const nlohmann::json& f = j.at(field_name);
            if(f.type() != field_type)
            {
                throw createLogicError(
                        fmt::format("json field '{0:s}' should be of type '{1:s}' however is instead of type '{2:s}'",
                                    field_name,
                                    magic_enum::enum_name(field_type),
                                    magic_enum::enum_name(f.type())),
                        location);
            }
        }
    }
}  // namespace traits::json_ext
