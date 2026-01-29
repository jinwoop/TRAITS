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
#include "traits/task_allocation/normalized_schedule_quality.hpp"

#include "traits/common/utilities/logger.hpp"

namespace traits
{
    NormalizedScheduleQuality::NormalizedScheduleQuality(
        const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
        const std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)>& on_failure,
        std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> on_success)
        : m_problem_inputs(problem_inputs)
        , m_create_scheduler(
              [](const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
              {
                  return std::make_shared<TraitsMilpScheduler>(problem_inputs);
              })
        , m_on_failure(on_failure)
        , m_on_success(on_success)
    {}

    NormalizedScheduleQuality::NormalizedScheduleQuality(
        const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
        const std::function<std::shared_ptr<TraitsSchedulerBase>(const std::shared_ptr<const TraitsSchedulerProblemInputs>&)>&
            create_scheduler,
        const std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)>& on_failure,
        std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> on_success)
        : m_problem_inputs(problem_inputs)
        , m_create_scheduler(create_scheduler)
        , m_on_failure(on_failure)
        , m_on_success(on_success)
    {}

    float NormalizedScheduleQuality::operator()(const std::shared_ptr<TraitsIncrementalTaskAllocationNode>& node) const
    {
        return operator()(node.get());
    }

    float NormalizedScheduleQuality::operator()(TraitsIncrementalTaskAllocationNode* node) const
    {
        return computeMakespanRatio(node);
    }

    float NormalizedScheduleQuality::computeMakespanRatio(TraitsIncrementalTaskAllocationNode* node) const
    {
        // Calculate the Makespan
        auto scheduler_problem_inputs = std::make_shared<TraitsSchedulerProblemInputs>(m_problem_inputs, node->allocation());
        auto scheduler                = m_create_scheduler(scheduler_problem_inputs);
        std::shared_ptr<const TraitsSchedulerResult> result = scheduler->solve();
        if(result->failed())
        {
            m_on_failure(result);
            node->setSchedule(nullptr);
            return std::numeric_limits<float>::infinity();
        }

        m_on_success(result);
        node->setSchedule(result->schedule());

        return node->schedule()->objectiveRatio();
    }

    float NormalizedScheduleQuality::computeMakespan(TraitsIncrementalTaskAllocationNode* node) const
    {
        // Calculate the Makespan
        auto scheduler_problem_inputs = std::make_shared<TraitsSchedulerProblemInputs>(m_problem_inputs, node->allocation());
        auto scheduler                = m_create_scheduler(scheduler_problem_inputs);
        std::shared_ptr<const TraitsSchedulerResult> result = scheduler->solve();
        if(result->failed())
        {
            m_on_failure(result);
            node->setSchedule(nullptr);
            return std::numeric_limits<float>::infinity();
        }

        m_on_success(result);
        node->setSchedule(result->schedule());
        return node->schedule()->makespan();
    }
}  // namespace traits