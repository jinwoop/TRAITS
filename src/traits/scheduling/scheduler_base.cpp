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
#include "traits/scheduling/scheduler_base.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/timer_runner.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"

namespace traits
{
    unsigned int TraitsSchedulerBase::s_num_failures = 0;

    TraitsSchedulerBase::TraitsSchedulerBase(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    std::shared_ptr<const TraitsSchedulerResult> TraitsSchedulerBase::solve()
    {
        TimerRunner timer_runner(constants::k_scheduling_time);
        return computeSchedule();
    }

    unsigned int TraitsSchedulerBase::numFailures()
    {
        return s_num_failures;
    }
}  // namespace traits