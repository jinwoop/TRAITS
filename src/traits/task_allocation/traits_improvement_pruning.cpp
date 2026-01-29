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
#include "traits/task_allocation/traits_improvement_pruning.hpp"

// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/common/utilities/logger.hpp"

namespace traits
{
    TraitsTraitsImprovementPruning::TraitsTraitsImprovementPruning(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool TraitsTraitsImprovementPruning::operator()(const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& node) const {

        // Actually below is wrong: this is the pre-pruning
        // CRUCIAL: don't do pre-pruning, but just check if child node improves
        // This check is irrelevant to the infeasibility

        if (node->parent() == nullptr) {
            return false;
        }

        float child_apr = TraitAndRatesMistmatch(node->allocation(),
                                                     m_problem_inputs->desiredTraitsMatrix(),
                                                     m_problem_inputs->desiredTraitRatesMatrix(),
                                                     m_problem_inputs->maxTeamTraitsMatrix(),
                                                     m_problem_inputs->maxTeamTraitRatesMatrix(),
                                                     m_problem_inputs->tasks(),
                                                     m_problem_inputs->gamma());

        float parent_apr = TraitAndRatesMistmatch(node->parent()->allocation(),
                                                      m_problem_inputs->desiredTraitsMatrix(),
                                                      m_problem_inputs->desiredTraitRatesMatrix(),
                                                      m_problem_inputs->maxTeamTraitsMatrix(),
                                                      m_problem_inputs->maxTeamTraitRatesMatrix(),
                                                      m_problem_inputs->tasks(),
                                                      m_problem_inputs->gamma());

        if (child_apr - parent_apr >= 0.0f) {  // means added robot is useless (>=)
            return true;
        }
        return false;
    }
}  // namespace traits
