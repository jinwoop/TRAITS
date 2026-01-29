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
#include <fmt/core.h>
#include "traits/task_allocation/allocation_percentage_remaining.hpp"

// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/time_keeper.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"
#include "traits/task_allocation/nlp/trait_distributor_result.hpp"
#include "traits/task_allocation/nlp/trait_distributor.hpp"

namespace traits {
    TraitsAllocationPercentageRemaining::TraitsAllocationPercentageRemaining(
            const std::shared_ptr<const TraitsProblemInputs> &problem_inputs)
            : m_problem_inputs(problem_inputs), m_desired_traits_sum(problem_inputs->desiredTraitsMatrix().sum()),
              m_desired_trait_rates_sum(problem_inputs->desiredTraitRatesMatrix().sum()),
              m_gamma(problem_inputs->gamma())
    {}

    float TraitsAllocationPercentageRemaining::operator()(
            const std::shared_ptr<TraitsIncrementalTaskAllocationNode> &node) const {
        TimerRunner timer_runner(constants::k_task_allocation_time);
        const Eigen::MatrixXf &allocation = node->allocation();

        if (allocation.sum() < 0.5f) {
            return 1.0f;
        }

        TraitDistributor td = TraitDistributor(allocation, m_problem_inputs);
        std::shared_ptr<const TraitDistributorResult> result = td.solve();
        node->setTaskTraitAllocation(result->taskTraitAllocation());

        if (node->taskTraitAllocation() == nullptr) {
            return std::numeric_limits<float>::infinity();
        }

        const float traits_mismatch_error = node->taskTraitAllocation()->trait_deficiencies();
        const float trait_rates_mismatch_error = node->taskTraitAllocation()->trait_rate_deficiencies();

        float apr = m_gamma * (traits_mismatch_error / m_desired_traits_sum);

        if (m_desired_trait_rates_sum > 0.0f) {
            apr += (1.0f - m_gamma) * (trait_rates_mismatch_error / m_desired_trait_rates_sum);
        }

        return apr;
    }
};