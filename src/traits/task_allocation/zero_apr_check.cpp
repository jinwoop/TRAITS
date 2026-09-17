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
#include "traits/task_allocation/zero_apr_check.hpp"
#include "traits/common/utilities/logger.hpp"

// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

namespace traits
{
    TraitsZeroAprCheck::TraitsZeroAprCheck(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
        : m_problem_inputs(problem_inputs)
    {}

    bool TraitsZeroAprCheck::operator()(const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& node) const
    {
        // The search hands back a null node when it exhausts the open set or times out.
        if (node == nullptr) {
            return false;
        }

        // A
        const Eigen::MatrixXf& allocation = node->allocation();

        // if root node
        if (allocation.sum() < 0.5f) {
            return false;  // not a Zero APR
        }

        // E
        if (node->taskTraitAllocation() == nullptr || node->schedule() == nullptr) {
            return false;
        }

        // NLP Trait Distributor and MILP Scheduler worked.
        const float traits_mismatch_error = node->taskTraitAllocation()->trait_deficiencies();
        const float trait_rates_mismatch_error = node->taskTraitAllocation()->trait_rate_deficiencies();

        return traits_mismatch_error + trait_rates_mismatch_error < 1e-4f;
    }

    std::vector<float> TraitsZeroAprCheck::tasksCompletionStatus(
            const std::shared_ptr<const TraitsIncrementalTaskAllocationNode> &node) const
    {
        // The search hands back a null node when it exhausts the open set or times out.
        if (node == nullptr) {
            return std::vector<float>();
        }

        // A
        const Eigen::MatrixXf& allocation = node->allocation();

        // E
        Eigen::MatrixXf traits_mismatch_matrix = traitsMismatchMatrix(*m_problem_inputs->robotTraitsMatrixReduction(),
                                                                      allocation,
                                                                      m_problem_inputs->desiredTraitsMatrix(),
                                                                      m_problem_inputs->teamTraitsMatrix());

        Eigen::VectorXf traits_mismatch_row_sum = traits_mismatch_matrix.rowwise().sum();
        std::vector<float> task_apr_status;
        task_apr_status.reserve(traits_mismatch_matrix.rows());
        for (int i = 0; i < traits_mismatch_row_sum.size(); ++i){
            task_apr_status.push_back(traits_mismatch_row_sum.coeff(i));
        }

        return task_apr_status;
    }
}  // namespace traits