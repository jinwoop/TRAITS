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
#include "traits/common/search/heuristic_base.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"

namespace traits
{
    class TraitsProblemInputs;
    class TraitDistributor;
    class TraitDistributorResult;
    class TaskTraitAllocation;

    /*!
     * Evaluates an allocation by computing the percentage of the desired traits left unsatisfied
     *
     * \see TRAITS
     */
    class TraitsAllocationPercentageRemaining : public HeuristicBase<TraitsIncrementalTaskAllocationNode>
    {
       public:
        //! Constructor
        TraitsAllocationPercentageRemaining(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs);

        //! \returns The percentage of the desired traits left unsatisfied by the allocation in \p node
        [[nodiscard]] float operator()(const std::shared_ptr<TraitsIncrementalTaskAllocationNode>& node) const final override;

       private:
        std::shared_ptr<const TraitsProblemInputs> m_problem_inputs;
        float m_desired_traits_sum;
        float m_desired_trait_rates_sum;
        float m_gamma;
    };
}  // namespace traits