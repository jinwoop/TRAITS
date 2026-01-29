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
#include "traits/task_allocation/dynamic_time_extended_task_allocation_quality.hpp"

namespace traits
{
    DynamicTimeExtendedTaskAllocationQuality::DynamicTimeExtendedTaskAllocationQuality(
        const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
        const std::shared_ptr<TraitsAllocationPercentageRemaining>& apr,
        const std::shared_ptr<NormalizedScheduleQuality>& nsq)
        : m_apr{apr != nullptr ? apr : std::make_shared<const TraitsAllocationPercentageRemaining>(problem_inputs)}
        , m_nsq{nsq != nullptr ? nsq : std::make_shared<const NormalizedScheduleQuality>(problem_inputs)}
    {
        m_alpha = problem_inputs->alpha();
    }

    DynamicTimeExtendedTaskAllocationQuality::DynamicTimeExtendedTaskAllocationQuality(
        const detail::DynamicTimeExtendedTaskAllocationQualityParameters& parameters)
        : m_alpha(parameters.alpha)
        , m_apr{parameters.apr != nullptr
                    ? parameters.apr
                    : std::make_shared<const TraitsAllocationPercentageRemaining>(parameters.problem_inputs)}
        , m_nsq{parameters.nsq != nullptr
                    ? parameters.nsq
                    : std::make_shared<const NormalizedScheduleQuality>(parameters.problem_inputs)}
    {}

    DynamicTimeExtendedTaskAllocationQuality::DynamicTimeExtendedTaskAllocationQuality(
        detail::DynamicTimeExtendedTaskAllocationQualityParameters&& parameters)
        : m_alpha(parameters.alpha)
        , m_apr{parameters.apr != nullptr
                    ? std::move(parameters.apr)
                    : std::make_shared<const TraitsAllocationPercentageRemaining>(parameters.problem_inputs)}
        , m_nsq{parameters.nsq != nullptr
                    ? std::move(parameters.nsq)
                    : std::make_shared<const NormalizedScheduleQuality>(parameters.problem_inputs)}
    {}

    float DynamicTimeExtendedTaskAllocationQuality::operator()(
        const std::shared_ptr<TraitsIncrementalTaskAllocationNode>& node) const
    {
        // for UB and LB TODO: REMOVE?
        if (node->apr() < -0.5f)
        {
            node->set_apr(m_apr->operator()(node));
        }
        if (node->nsq() < -0.5f)
        {
            node->set_nsq(m_nsq->operator()(node));
        }

        // only for root node
        if (node->parent() == nullptr)
        {
            return std::numeric_limits<float>::infinity();
        }

        // TETAQ
        return m_alpha * node->apr() + (1.0f - m_alpha) * node->nsq();
    }
}  // namespace traits