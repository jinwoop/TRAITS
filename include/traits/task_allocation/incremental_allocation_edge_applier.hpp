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
#include "traits/common/search/edge_applier_base.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"

namespace traits
{
    class TraitsProblemInputs;

    /*!
     * Functor to add an edge representing the allocation of a specific robot to a specific task
     */
    class IncrementalAllocationEdgeApplier : public EdgeApplierBase<TraitsIncrementalTaskAllocationNode>
    {
       public:
        /*!
         * Constructor
         *
         * \param assignment The assignment this edge represents
         * \param problem_inputs Inputs to the task allocation problem
         */
        IncrementalAllocationEdgeApplier(const Assignment& assignment,
                                             const std::shared_ptr<const TraitsProblemInputs>& problem_inputs);

        //! \returns Whether the edge this edge applier represents can be added to \p base
        [[nodiscard]] bool isApplicable(
            const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& base) const final override;

        //! \returns The succeeding node if this edge applier can be applied, nullptr otherwise
        [[nodiscard]] std::shared_ptr<TraitsIncrementalTaskAllocationNode> apply(
            const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& base) const final override;

       private:
        Assignment m_assignment;
        std::shared_ptr<const TraitsProblemInputs> m_problem_inputs;
    };
}  // namespace traits