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
#include "traits/task_allocation/incremental_task_allocation_node.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/hash_extension.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/configurations/configuration_base.hpp"
#include "traits/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/robot.hpp"
#include "traits/scheduling/milp/milp_scheduler.hpp"
#include "traits/scheduling/milp/schedule.hpp"
#include "traits/scheduling/scheduler_result.hpp"
#include "traits/task.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

namespace traits
{
    unsigned int TraitsIncrementalTaskAllocationNode::s_next_id = 0;

    TraitsIncrementalTaskAllocationNode::TraitsIncrementalTaskAllocationNode(const MatrixDimensions& dimensions)
        : Base_(s_next_id++, nullptr)
        , m_last_assigment(std::nullopt)
        , m_matrix_dimensions(dimensions)
        , m_schedule(nullptr)
        , m_task_trait_allocations(nullptr)
        , m_apr(-1.0f)
        , m_nsq(-1.0f)
    {}

    TraitsIncrementalTaskAllocationNode::TraitsIncrementalTaskAllocationNode(const MatrixDimensions& dimensions,
                                                                         float apr)
            : Base_(s_next_id++, nullptr)
            , m_last_assigment(std::nullopt)
            , m_matrix_dimensions(dimensions)
            , m_schedule(nullptr)
            , m_task_trait_allocations(nullptr)
            , m_apr(apr)
            , m_nsq(-1.0f)
    {}

    TraitsIncrementalTaskAllocationNode::TraitsIncrementalTaskAllocationNode(
        const Assignment& assignment,
        const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& parent)
        : Base_(s_next_id++, parent)
        , m_last_assigment(assignment)
        , m_matrix_dimensions(std::nullopt)
        , m_schedule(nullptr)
        , m_task_trait_allocations(nullptr)
        , m_apr(-1.0f)
        , m_nsq(-1.0f)
    {
        assert(parent);
    }

    const MatrixDimensions& TraitsIncrementalTaskAllocationNode::matrixDimensions() const
    {
        if(m_matrix_dimensions.has_value())
        {
            return m_matrix_dimensions.value();
        }
        return m_parent->matrixDimensions();
    }

    Eigen::MatrixXf TraitsIncrementalTaskAllocationNode::allocation() const
    {
        const MatrixDimensions& dimensions = matrixDimensions();

        // Allocation matrix is M X N (number_of_tasks X number_of_robots)
        Eigen::MatrixXf matrix(dimensions.height, dimensions.width);  // (# rows, # columns)
        matrix.setZero();
        if(m_last_assigment == std::nullopt)
        {
            return matrix;
        }

        matrix(m_last_assigment->task, m_last_assigment->robot) = 1.0f;

        std::shared_ptr<const TraitsIncrementalTaskAllocationNode> parent;
        for(parent = m_parent; parent->m_last_assigment != std::nullopt; parent = parent->parent())
        {
            matrix(parent->m_last_assigment->task, parent->m_last_assigment->robot) = 1.0f;
        }
        return matrix;
    }

    unsigned int TraitsIncrementalTaskAllocationNode::hash() const
    {
        return std::hash<Eigen::MatrixXf>()(allocation());
    }

    nlohmann::json TraitsIncrementalTaskAllocationNode::serializeToJson(
        const std::shared_ptr<const ProblemInputs>& problem_inputs) const
    {
        if(m_schedule)
        {
            nlohmann::json j_rv;
            j_rv = m_schedule->serializeToJson(std::make_shared<TraitsSchedulerProblemInputs>(
                std::dynamic_pointer_cast<const TraitsProblemInputs>(problem_inputs),
                allocation()));

            return j_rv;
        }

        // No schedule already computed
        auto scheduler_problem_inputs = std::make_shared<TraitsSchedulerProblemInputs>(
            std::dynamic_pointer_cast<const TraitsProblemInputs>(problem_inputs),
            allocation());

        TraitsMilpScheduler scheduler(scheduler_problem_inputs);
        std::shared_ptr<const TraitsSchedulerResult> result = scheduler.solve();

        if(result->success())
        {
            return result->schedule()->serializeToJson(scheduler_problem_inputs);
        }

        // Creates null json
        return nullptr;
    }
}  // namespace traits