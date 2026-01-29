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
#include "traits/scheduling/scheduler_result.hpp"

// Global
#include <cassert>

namespace traits
{
    TraitsSchedulerResult::TraitsSchedulerResult(const std::shared_ptr<const TraitsSchedule>& schedule)
        : m_schedule(schedule)
        , m_failure_reason(nullptr)
    {
        assert(schedule);
    }

    TraitsSchedulerResult::TraitsSchedulerResult(const std::shared_ptr<const FailureReason>& failure_reason)
        : m_schedule(nullptr)
        , m_failure_reason(failure_reason)
    {
        assert(failure_reason);
    }
}  // namespace traits