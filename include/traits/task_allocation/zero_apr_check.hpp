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

// Local
#include "traits/common/search/goal_check_base.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"

namespace traits
{
    // forward declarations
    class TaskTraitAllocation;
    // forward declarations end

    //! \brief Checks that a task allocation satisfies the desired traits
    class TraitsZeroAprCheck : public GoalCheckBase<TraitsIncrementalTaskAllocationNode>
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param desired_trait_matrix
         * \param robot_trait_matrix
         */
        explicit TraitsZeroAprCheck(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs);

        //! \returns Whether \p node satisfies the desired traits matrix
        [[nodiscard]] bool operator()(
            const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& node) const final override;

        [[nodiscard]] std::vector<float> tasksCompletionStatus(
                const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& node) const;

       private:
        std::shared_ptr<const TraitsProblemInputs> m_problem_inputs;
    };
}  // namespace traits