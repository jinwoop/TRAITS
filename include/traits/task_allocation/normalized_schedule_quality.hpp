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
// Local
#include "traits/common/search/heuristic_base.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/milp/milp_scheduler.hpp"
#include "traits/scheduling/scheduler_base.hpp"
#include "traits/scheduling/scheduler_result.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"

namespace traits
{
    /*!
     * Evaluates an allocation based on the quality of the makespan from the associated schedule
     */
    class NormalizedScheduleQuality : public HeuristicBase<TraitsIncrementalTaskAllocationNode>
    {
       public:
        //! \brief Constructor
        explicit NormalizedScheduleQuality(
            const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
            const std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)>& on_failure =
                [](const std::shared_ptr<const TraitsSchedulerResult>&) {},
            std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> on_success =
                [](const std::shared_ptr<const TraitsSchedulerResult>&) {});

        //! \brief Constructor
        explicit NormalizedScheduleQuality(
            const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
            const std::function<std::shared_ptr<TraitsSchedulerBase>(const std::shared_ptr<const TraitsSchedulerProblemInputs>&)>&
                create_scheduler,
            const std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)>& on_failure =
                [](const std::shared_ptr<const TraitsSchedulerResult>&) {},
            std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> on_success =
                [](const std::shared_ptr<const TraitsSchedulerResult>&) {});

        //! \returns The quality of the makespan of the associated schedule
        [[nodiscard]] float operator()(const std::shared_ptr<TraitsIncrementalTaskAllocationNode>& node) const final override;

        //! \returns The quality of the makespan of the associated schedule
        [[nodiscard]] virtual float operator()(TraitsIncrementalTaskAllocationNode* node) const;

       protected:
        //! \returns The makespan for the associated schedule of \p node
        [[nodiscard]] virtual float computeMakespan(TraitsIncrementalTaskAllocationNode* node) const;

        [[nodiscard]] float computeMakespanRatio(TraitsIncrementalTaskAllocationNode* node) const;

        std::shared_ptr<const TraitsProblemInputs> m_problem_inputs;
        std::function<std::shared_ptr<TraitsSchedulerBase>(const std::shared_ptr<const TraitsSchedulerProblemInputs>&)>
            m_create_scheduler;
        std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> m_on_failure;
        std::function<void(const std::shared_ptr<const TraitsSchedulerResult>&)> m_on_success;
    };
}  // namespace traits