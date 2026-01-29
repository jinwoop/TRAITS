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
#include "traits/task_allocation/infeasible_pruning.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/common/utilities/logger.hpp"

namespace traits
{
    TraitsInfeasiblePruning::TraitsInfeasiblePruning(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool TraitsInfeasiblePruning::operator()(const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& node) const
    {
        // return true only for infeasible plans
        if (node->taskTraitAllocation() == nullptr ||  // APR fail
            node->schedule() == nullptr)  // NSQ fail
        {
            return true;   // Prune if true -- i.e., worse than the parent node
        }

        // Never prune child based on comparison between parent's apr
        // because it could be on correct path but apr may temporarily increase.
        // in the trait improvement phase, that APR is based on max traits not true provision -- like necessary condition.

        return false;
    }
}  // namespace traits