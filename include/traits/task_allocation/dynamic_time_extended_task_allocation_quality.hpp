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
#pragma once

// Global
#include <memory>
// Local
#include "traits/common/search/heuristic_base.hpp"
#include "traits/task_allocation/allocation_percentage_remaining.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/task_allocation/normalized_schedule_quality.hpp"

namespace traits
{
    class TraitsProblemInputs;

    namespace detail
    {
        struct DynamicTimeExtendedTaskAllocationQualityParameters
        {
            std::shared_ptr<const TraitsProblemInputs> problem_inputs;
            float alpha                                        = 0.5;
            std::shared_ptr<TraitsAllocationPercentageRemaining> apr = nullptr;
            std::shared_ptr<NormalizedScheduleQuality> nsq     = nullptr;
        };
    }  // namespace detail

    /*!
     * \brief Computes the Time Extended Task Allocation Quality heuristic
     *
     * This heuristic is a convex combination of Allocation Percentage Remaining and Normalized Schedule Quality
     *
     * \see AllocationPercentageRemaining
     * \see NormalizedScheduleQuality
     * \see TRAITS
     *
     */
    class DynamicTimeExtendedTaskAllocationQuality : public HeuristicBase<TraitsIncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        explicit DynamicTimeExtendedTaskAllocationQuality(
                const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                const std::shared_ptr<TraitsAllocationPercentageRemaining>& apr = nullptr,
                const std::shared_ptr<NormalizedScheduleQuality>& nsq     = nullptr);

        //! Constructor
        explicit DynamicTimeExtendedTaskAllocationQuality(
            const detail::DynamicTimeExtendedTaskAllocationQualityParameters& parameters);

        //! Constructor
        explicit DynamicTimeExtendedTaskAllocationQuality(detail::DynamicTimeExtendedTaskAllocationQualityParameters&& parameters);

        //! \returns A combination of APR and RSQ heuristics
        [[nodiscard]] float operator()(const std::shared_ptr<TraitsIncrementalTaskAllocationNode>& node) const final override;

    protected:
        float m_alpha;
        std::shared_ptr<const TraitsAllocationPercentageRemaining> m_apr;
        std::shared_ptr<const NormalizedScheduleQuality> m_nsq;
    };
}  // namespace traits