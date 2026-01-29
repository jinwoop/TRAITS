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
#include "traits/scheduling/deadline_base.hpp"

// External
#include <fmt/format.h>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/scheduling/absolute_deadline.hpp"
#include "traits/scheduling/relative_deadline.hpp"

namespace traits
{
    DeadlineBase::DeadlineBase(DeadlineType t)
            : m_type(t)
    {}
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<const traits::DeadlineBase>
    adl_serializer<std::shared_ptr<const traits::DeadlineBase>>::from_json(const json& j)
    {
        return adl_serializer<std::shared_ptr<traits::DeadlineBase>>::from_json(j);
    }

    std::shared_ptr<traits::DeadlineBase> adl_serializer<std::shared_ptr<traits::DeadlineBase>>::from_json(
            const json& j)
    {
        traits::json_ext::validateJson(j, {{traits::constants::k_deadline_type, nlohmann::json::value_t::string}});

        const traits::DeadlineType deadline_type = j.at(traits::constants::k_deadline_type);
        switch(deadline_type)
        {
            case traits::DeadlineType::e_absolute:
            {
                return j.get<std::shared_ptr<traits::AbsoluteDeadline>>();
            }
            case traits::DeadlineType::e_relative:
            {
                return j.get<std::shared_ptr<traits::RelativeDeadline>>();
            }
            default:
            {
                throw traits::createLogicError(
                        fmt::format("Unknown Deadline Type: {0:s}",
                                    j.at(traits::constants::k_deadline_type).get<std::string>()));
            }
        }
    }

    void adl_serializer<std::shared_ptr<const traits::DeadlineBase>>::to_json(
            json& j,
            const std::shared_ptr<const traits::DeadlineBase>& d)
    {
        switch(d->deadlineType())
        {
            case traits::DeadlineType::e_absolute:
            {
                j = std::dynamic_pointer_cast<const traits::AbsoluteDeadline>(d);
                return;
            }
            case traits::DeadlineType::e_relative:
            {
                j = std::dynamic_pointer_cast<const traits::RelativeDeadline>(d);
                return;
            }
            default:
            {
                throw traits::createLogicError("Unknown ConfigurationType");
            }
        }
    }

    void adl_serializer<std::shared_ptr<traits::DeadlineBase>>::to_json(
            json& j,
            const std::shared_ptr<traits::DeadlineBase>& d)
    {
        adl_serializer<std::shared_ptr<const traits::DeadlineBase>>::to_json(j, d);
    }
}  // namespace nlohmann