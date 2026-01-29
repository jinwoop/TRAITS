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
#include "traits/scheduling/relative_deadline.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/json_extension.hpp"

namespace traits
{
    RelativeDeadline::RelativeDeadline(const TimePoint& predecessor, const TimePoint& successor, float bound)
            : DeadlineBase(DeadlineType::e_relative)
            , m_predecessor(predecessor)
            , m_successor(successor)
            , m_bound(bound)
    {}
}  // namespace grstapse

namespace nlohmann {
    std::shared_ptr<const traits::RelativeDeadline>
    adl_serializer<std::shared_ptr<const traits::RelativeDeadline> >::from_json(const json &j) {
        return adl_serializer<std::shared_ptr<traits::RelativeDeadline> >::from_json(j);
    }

    std::shared_ptr<traits::RelativeDeadline> adl_serializer<std::shared_ptr<traits::RelativeDeadline> >::from_json(
        const json &j) {
        traits::json_ext::validateJson(j,
                                       {
                                           {traits::constants::k_deadline_type, nlohmann::json::value_t::string},
                                           {traits::constants::k_predecessor, nlohmann::json::value_t::object},
                                           {traits::constants::k_successor, nlohmann::json::value_t::object},
                                           {traits::constants::k_bound, nlohmann::json::value_t::number_float}
                                       });

        const traits::TimePoint predecessor = j.at(traits::constants::k_predecessor);
        const traits::TimePoint successor = j.at(traits::constants::k_successor);
        const float bound = j.at(traits::constants::k_bound);

        return std::make_shared<traits::RelativeDeadline>(predecessor, successor, bound);
    }

    void adl_serializer<std::shared_ptr<const traits::RelativeDeadline> >::to_json(
        json &j,
        const std::shared_ptr<const traits::RelativeDeadline> &d) {
        j = {
            {traits::constants::k_deadline_type, d->deadlineType()},
            {traits::constants::k_bound, d->bound()},
            {traits::constants::k_predecessor, d->predecessor()},
            {traits::constants::k_successor, d->successor()}
        };
    }

    void adl_serializer<std::shared_ptr<traits::RelativeDeadline> >::to_json(
        json &j,
        const std::shared_ptr<traits::RelativeDeadline> &d) {
        adl_serializer<std::shared_ptr<const traits::RelativeDeadline> >::to_json(j, d);
    }
} // namespace nlohmann
