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
#include "traits/scheduling/milp/task_info.hpp"

// Global
#include <string_view>
// External
#include <range/v3/all.hpp>
// Local
#include "traits/common/milp/milp_utilities.hpp"
#include "traits/common/utilities/compound_failure_reason.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/common/utilities/std_extension.hpp"
#include "traits/scheduling/initial_transition_failure.hpp"
#include "traits/scheduling/milp/name_scheme.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/scheduling/scheduler_motion_planner_interface.hpp"
#include "traits/scheduling/task_duration_failure.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/task.hpp"

namespace traits
{
    TraitsTaskInfo::TraitsTaskInfo(TraitsCoalitionView coalition,
                             unsigned int plan_task_nr,
                             const std::shared_ptr<const Task>& task,
                             const std::shared_ptr<const NameScheme>& name_scheme,
                             const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& motion_planner_interface,
                             const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
        : m_plan_task_nr(plan_task_nr)
        , m_task(task)
        , m_name_scheme(name_scheme)
        , m_motion_planner_interface(motion_planner_interface)
        , m_duration(0.0f)
        , m_lower_bound(0.0f)
        , m_upper_bound(std::numeric_limits<float>::infinity())
        , m_problem_inputs(problem_inputs)
    {
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            m_coalition[robot] =
                std::pair(TransitionComputationStatus::e_none, std::numeric_limits<float>::quiet_NaN());
        }
    }

    std::shared_ptr<const FailureReason> TraitsTaskInfo::setupData()
    {
        // Lower Bound Calculation
        if(m_coalition.empty())
        {
            m_lower_bound = 0.0f;
            m_duration    = m_task->staticDuration();
            if (m_task->fastestSpeciesSpeed() > 0.0f)
            {
                m_duration = m_task->computeDurationLBHeuristic();
            }
            return nullptr;
        }

        const std::shared_ptr<const ConfigurationBase>& initial_configuration  = m_task->initialConfiguration();
        const std::shared_ptr<const ConfigurationBase>& terminal_configuration = m_task->terminalConfiguration();
        std::vector<std::shared_ptr<const Robot>> coalition;
        coalition.reserve(m_coalition.size());
        for(auto& [robot, transition_status]: m_coalition)
        {
            coalition.push_back(robot);

            // Compute Initial Transition Data
            float initial_transition_duration;
            if(m_motion_planner_interface->isInitialTransitionMemoized(initial_configuration, robot))
            {
                initial_transition_duration =
                    m_motion_planner_interface->computeDuration(robot->initialConfiguration(),
                                                                initial_configuration,
                                                                robot->species(),
                                                                robot->inter_transition_velocity());
                if(initial_transition_duration < 0.0f)
                {
                    return std::shared_ptr<const InitialTransitionFailure>(
                        new InitialTransitionFailure{{.robot = robot->id(), .task = m_plan_task_nr}});
                }
                transition_status.first = TransitionComputationStatus::e_success;
            }

            transition_status.second = initial_transition_duration;
            m_lower_bound            = std::max(m_lower_bound, initial_transition_duration);
        }

        // this calculates how long coalition takes to move within a task.
        m_duration = m_motion_planner_interface->computeTaskDuration(m_task, coalition, m_problem_inputs);
        if(m_duration < 0.0f)
        {
            std::vector<std::shared_ptr<const FailureReason>> reasons;
            reasons.reserve(m_coalition.size());
            for(const std::shared_ptr<const Robot>& robot: coalition)
            {
                reasons.push_back(std::shared_ptr<TaskDurationFailure>(
                    new TaskDurationFailure{{.species = robot->species()->name(), .task = m_plan_task_nr}}));
            }
            return std::make_shared<CompoundFailureReason>(reasons);
        }
        return nullptr;
    }

    void TraitsTaskInfo::createTimePointVariables(GRBModel& model)
    {
        m_start_time_point = model.addVar(-GRB_INFINITY,
                                          GRB_INFINITY,
                                          0.0,
                                          GRB_CONTINUOUS,
                                          m_name_scheme->createTaskStartName(m_plan_task_nr));

        m_end_time_point = model.addVar(-GRB_INFINITY,
                                        GRB_INFINITY,
                                        0.0,
                                        GRB_CONTINUOUS,
                                        m_name_scheme->createTaskEndName(m_plan_task_nr));
    }

    void TraitsTaskInfo::createTaskEndTimeConstraint(GRBModel& model)
    {
        m_task_end_time_constraint =
                model.addConstr(m_start_time_point - m_end_time_point == -m_duration,
                                m_name_scheme->createTaskEndTimeConstraintName(m_plan_task_nr));
    }

    void TraitsTaskInfo::createLowerBoundConstraint(GRBModel& model)
    {
        m_lower_bound_constraint =
            model.addConstr(-m_start_time_point <= -m_lower_bound,
                            m_name_scheme->createTaskStartLowerBoundConstraintName(m_plan_task_nr));
    }

    void TraitsTaskInfo::createUpperBoundConstraint(GRBModel& model)
    {
        m_upper_bound_constraint =
                model.addConstr(m_end_time_point <= m_upper_bound,
                                m_name_scheme->createTaskEndUpperBoundConstraintName(m_plan_task_nr));
    }

    UpdateModelResult TraitsTaskInfo::updateLowerBound(const std::shared_ptr<const Robot>& robot)
    {
        std::pair<TransitionComputationStatus, float>& transition_status = m_coalition[robot];
        if(transition_status.first == TransitionComputationStatus::e_success)
        {
            return UpdateModelResult(UpdateModelResultType::e_no_update);
        }

        const std::shared_ptr<const ConfigurationBase>& initial_configuration = m_task->initialConfiguration();
        const float initial_transition_duration =
            m_motion_planner_interface->computeInitialTransitionDuration(initial_configuration, robot);
        if(initial_transition_duration < 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const InitialTransitionFailure>(
                new InitialTransitionFailure{{.robot = robot->id(), .task = m_plan_task_nr}}));
        }

        transition_status.first  = TransitionComputationStatus::e_success;
        transition_status.second = initial_transition_duration;
        if(initial_transition_duration > m_lower_bound)
        {
            m_lower_bound = initial_transition_duration;
            m_lower_bound_constraint.set(GRB_DoubleAttr_RHS, -m_lower_bound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }

        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    UpdateModelResult TraitsTaskInfo::updateLowerBoundHeuristic(
            const std::shared_ptr<const Robot>& robot,
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
    {
        float max_distance = -1.0f;
        for (const std::shared_ptr<const Robot>& a_robot : problem_inputs->getTraitsProblemInputs()->robots()) {
            max_distance = std::max(max_distance,
                                    m_motion_planner_interface->computeMaxDistance(a_robot->initialConfiguration(),
                                                                                   m_task->initialConfiguration(),
                                                                                   problem_inputs));
        }

        if (max_distance <= 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const InitialTransitionFailure>(
                    new InitialTransitionFailure{{.robot = robot->id(), .task = m_plan_task_nr}}));
        }

        const float initial_transition_duration = max_distance / m_task->slowestSpeciesSpeed();

        if(initial_transition_duration > m_lower_bound)
        {
            m_lower_bound = initial_transition_duration;
            m_lower_bound_constraint.set(GRB_DoubleAttr_RHS, -m_lower_bound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }
        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    UpdateModelResult TraitsTaskInfo::updateLowerBoundHeuristicForce(float lower_bound)
    {
        if (lower_bound <= 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const InitialTransitionFailure>(
                    new InitialTransitionFailure{{.robot = 0, .task = m_plan_task_nr}}));
        }

        if(lower_bound >= m_lower_bound)
        {
            m_lower_bound = lower_bound;
            m_lower_bound_constraint.set(GRB_DoubleAttr_RHS, -m_lower_bound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }
        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    UpdateModelResult TraitsTaskInfo::updateLowerBoundHeuristicMax(float lower_bound)
    {
        if (lower_bound <= 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const InitialTransitionFailure>(
                    new InitialTransitionFailure{{.robot = 0, .task = m_plan_task_nr}}));
        }

        if(lower_bound > m_lower_bound)
        {
            m_lower_bound = lower_bound;
            m_lower_bound_constraint.set(GRB_DoubleAttr_RHS, m_lower_bound); // (+)ve for maximization
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }
        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    UpdateModelResult TraitsTaskInfo::updateUpperBound(float bound)
    {
        m_upper_bound_constraint.set(GRB_DoubleAttr_RHS, bound);
        return UpdateModelResult(UpdateModelResultType::e_updated);
    }

    std::vector<std::shared_ptr<const Robot>> TraitsTaskInfo::coalition() const
    {
        return m_coalition | ranges::views::keys | ranges::to<std::vector<std::shared_ptr<const Robot>>>();
    }

    double TraitsTaskInfo::dualCut() const
    {
        const double eta = constraintDualValue(m_lower_bound_constraint);
        return m_lower_bound * eta;
    }

}  // namespace traits