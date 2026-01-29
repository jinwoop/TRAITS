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
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"

// External
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/robot.hpp"
#include "traits/task_allocation/task_allocation_math.hpp" //

namespace traits
{
    TraitsSchedulerProblemInputs::TraitsSchedulerProblemInputs(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                                                           const Eigen::MatrixXf& allocation,
                                                           SchedulerObjectiveType scheduler_objective_type)
        : m_traits_problem_inputs(problem_inputs)
        , m_mutex_constraints(computeMutexConstraints(allocation))
        , m_allocation(allocation)
        , m_schedule_objective_type(scheduler_objective_type)
    {}

    TraitsSchedulerProblemInputs::TraitsSchedulerProblemInputs(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                                                             Eigen::MatrixXf&& allocation,
                                                             SchedulerObjectiveType scheduler_objective_type)
        : m_traits_problem_inputs(problem_inputs)
        , m_mutex_constraints(computeMutexConstraints(allocation))
        , m_allocation(std::move(allocation))
        , m_schedule_objective_type(scheduler_objective_type)
    {}

    void TraitsSchedulerProblemInputs::validate() const
    {
        unsigned int num_plan_task = numberOfPlanTasks();
        for(const std::pair<unsigned int, unsigned int>& constraint: m_mutex_constraints)
        {
            if(constraint.first >= num_plan_task || constraint.second >= num_plan_task)
            {
                throw createLogicError("Precedence constraint out of range of the number of plan tasks");
            }
        }
    }

    // returns vector of robot indices in this coalition - task_nr.
    TraitsCoalitionView TraitsSchedulerProblemInputs::coalition(unsigned int task_nr) const
    {
        return ::ranges::views::iota(0u, numberOfRobots()) |
               ::ranges::views::filter(std::function(
                   [this, task_nr](unsigned int r) -> bool
                   {
                       return m_allocation(task_nr, r) > 0.5f;
                   })) |
               ::ranges::views::transform(std::function(
                   [this](unsigned int r) -> const std::shared_ptr<const Robot>&
                   {
                       return robot(r);
                   }));
    }

    // returns vector of robot indices involved in this transition.
    TraitsCoalitionView TraitsSchedulerProblemInputs::transitionCoalition(unsigned int i, unsigned int j) const
    {
        return ::ranges::views::iota(0u, numberOfRobots()) |
               ::ranges::views::filter(std::function(
                   [this, i, j](unsigned int r) -> bool
                   {
                       return m_allocation(i, r) > 0.5f && m_allocation(j, r);
                   })) |
               ::ranges::views::transform(std::function(
                   [this](unsigned int r) -> const std::shared_ptr<const Robot>&
                   {
                       return robot(r);
                   }));
    }
}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<traits::TraitsSchedulerProblemInputs>
    adl_serializer<std::shared_ptr<traits::TraitsSchedulerProblemInputs>>::from_json(const nlohmann::json& j)
    {
        auto traits_problem_inputs = j.get<std::shared_ptr<traits::TraitsProblemInputs>>();
        auto allocation           = j[traits::constants::k_allocation].get<Eigen::MatrixXf>();
        return std::make_shared<traits::TraitsSchedulerProblemInputs>(traits_problem_inputs, std::move(allocation));
    }
}  // namespace nlohmann