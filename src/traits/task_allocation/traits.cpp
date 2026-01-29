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
#include "traits/task_allocation/traits.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"

namespace traits
{
    TRAITS::TRAITS(
        const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
        const std::shared_ptr<const HeuristicBase<TraitsIncrementalTaskAllocationNode>>& heuristic,
        const std::shared_ptr<const SuccessorGeneratorBase<TraitsIncrementalTaskAllocationNode>>& successor_generator,
        const std::shared_ptr<const GoalCheckBase<TraitsIncrementalTaskAllocationNode>>& goal_check,
        const std::shared_ptr<const MemoizationBase<TraitsIncrementalTaskAllocationNode>>& memoization,
        const std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>>& pre_pruning_method,
        const std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>>& post_pruning_method)
        : Base_{problem_inputs->TraitsParameters(),
                {.heuristic           = heuristic,
                 .successor_generator = successor_generator,
                 .goal_check          = goal_check,
                 .memoization         = memoization,
                 .prepruning_method   = pre_pruning_method,
                 .postpruning_method  = post_pruning_method}}
        , m_problem_inputs(problem_inputs)
    {}

    bool TRAITS::isAllocatable() const
    {
        // N
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        // M
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();

        // A \in \R^{M \times N}
        Eigen::MatrixXf allocation = Eigen::MatrixXf::Ones(num_tasks, num_robots);

        return traitsMismatchError(*m_problem_inputs->robotTraitsMatrixReduction(),
                                   allocation,
                                   m_problem_inputs->desiredTraitsMatrix(),
                                   m_problem_inputs->teamTraitsMatrix()) == 0;
    }

    std::shared_ptr<TraitsIncrementalTaskAllocationNode> TRAITS::createRootNode()
    {
        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        const unsigned int num_tasks  = m_problem_inputs->numberOfPlanTasks();
        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        return std::make_shared<TraitsIncrementalTaskAllocationNode>(
                MatrixDimensions{.height = num_tasks, .width = num_robots}, 1.0f);  // set root apr 1.0
    }
}  // namespace traits