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
#include "traits/task_allocation/incremental_allocation_edge_applier.hpp"

// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"

namespace traits
{
    IncrementalAllocationEdgeApplier::IncrementalAllocationEdgeApplier(
        const Assignment& assignment,
        const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
        : m_assignment(assignment)
        , m_problem_inputs(problem_inputs)
    {}

    bool IncrementalAllocationEdgeApplier::isApplicable(
        const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& base) const
    {
        // If the assignment has already been added then ignore
        std::shared_ptr<const TraitsIncrementalTaskAllocationNode> parent;
        for(parent = base; parent != nullptr; parent = parent->parent())
        {
            if(const std::optional<Assignment>& last_assignment = parent->lastAssigment();
               last_assignment.has_value() && last_assignment.value() == m_assignment)
            {
                return false;
            }
        }

        return true;
    }

    std::shared_ptr<TraitsIncrementalTaskAllocationNode> IncrementalAllocationEdgeApplier::apply(
        const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& base) const
    {
        return std::make_shared<TraitsIncrementalTaskAllocationNode>(m_assignment, base);
    }
}  // namespace traits