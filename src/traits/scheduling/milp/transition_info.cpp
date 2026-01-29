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
#include "traits/scheduling/milp/transition_info.hpp"

// Local
#include "traits/common/milp/milp_utilities.hpp"
#include "traits/scheduling/milp/name_scheme.hpp"
#include "traits/scheduling/scheduler_motion_planner_interface.hpp"
#include "traits/scheduling/transition_failure.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"

namespace traits
{
    TraitsTransitionInfo::TraitsTransitionInfo(
        TraitsCoalitionView coalition,
        unsigned int predecessor_index,
        unsigned int successor_index,
        const std::shared_ptr<const ConfigurationBase>& initial_configuration,
        const std::shared_ptr<const ConfigurationBase>& terminal_configuration,
        const std::shared_ptr<const NameScheme>& name_scheme,
        const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& motion_planner_interface)
        : m_predecessor_index(predecessor_index)
        , m_successor_index(successor_index)
        , m_initial_configuration(initial_configuration)
        , m_terminal_configuration(terminal_configuration)
        , m_name_scheme(name_scheme)
        , m_motion_planner_interface(motion_planner_interface)
        , m_duration_lowerbound(0.0f)
        , m_duration_upperbound(0.0f)
    {
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            m_coalition[robot] =
                std::pair(TransitionComputationStatus::e_none, std::numeric_limits<float>::quiet_NaN());
        }
    }

    std::shared_ptr<const FailureReason> TraitsTransitionInfo::setupData()
    {
        for(auto& [robot, transition_status]: m_coalition)
        {
            // Compute Transition Data
            float transition_duration =
                m_motion_planner_interface->computeTransitionDuration(m_initial_configuration,
                    m_terminal_configuration, robot);

            if(transition_duration < 0.0f) {
                return std::shared_ptr<const TransitionFailure>(
                    new TransitionFailure{{.species                = robot->species()->name(),
                                           .predecessor_task_index = m_predecessor_index,
                                           .successor_task_index   = m_successor_index}});
            }

            transition_status.first = TransitionComputationStatus::e_success;
            transition_status.second = transition_duration;
            // i.e., phi_ij_min
            m_duration_lowerbound    = std::max(m_duration_lowerbound, transition_duration);
            // NOTE: redundant, but for readability
            // i.e., phi_ij_max
            m_duration_upperbound    = std::max(m_duration_upperbound, transition_duration);
        }
        return nullptr;
    }

    void TraitsTransitionInfo::createPrecedenceTransitionConstraint(GRBModel& model,
                                                                 GRBVar& predecessor,
                                                                 GRBVar& successor)
    {
        m_transition_name = m_name_scheme->createPrecedenceConstraintName(m_predecessor_index, m_successor_index);
        m_transition_constraint =
            model.addConstr(predecessor - successor <= -m_duration_lowerbound, m_transition_name);
    }

    void TraitsTransitionInfo::createMutexTransitionConstraint(GRBModel& model,
                                                            GRBVar& predecessor,
                                                            GRBVar& successor,
                                                            GRBLinExpr&& mutex_indicator_component)
    {
        m_transition_name       = m_name_scheme->createMutexConstraintName(m_predecessor_index, m_successor_index);
        m_transition_constraint = model.addConstr(
            predecessor - successor - mutex_indicator_component <= -m_duration_lowerbound, m_transition_name);
    }

    void TraitsTransitionInfo::createMutexTransitionConstraintHeuristic (GRBModel& model,
                                                                       GRBVar& predecessor,
                                                                       GRBVar& successor,
                                                                       GRBLinExpr&& mutex_indicator_component)
    {
        m_transition_name       = m_name_scheme->createMutexConstraintName(m_predecessor_index, m_successor_index);
        m_transition_constraint = model.addConstr(
                predecessor - successor - mutex_indicator_component <= -m_duration_lowerbound,
                m_transition_name);
    }

    UpdateModelResult TraitsTransitionInfo::updateLowerBound(const std::shared_ptr<const Robot>& robot)
    {
        std::pair<TransitionComputationStatus, float>& transition_status = m_coalition[robot];
        if(transition_status.first == TransitionComputationStatus::e_success)
        {
            return UpdateModelResult(UpdateModelResultType::e_no_update);
        }

        const float computed_transition_duration =
            m_motion_planner_interface->computeTransitionDuration(m_initial_configuration,
                                                                  m_terminal_configuration,
                                                                  robot);
        if(computed_transition_duration < 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const TransitionFailure>(
                new TransitionFailure{{.species                = robot->species()->name(),
                                       .predecessor_task_index = m_predecessor_index,
                                       .successor_task_index   = m_successor_index}}));
        }

        transition_status.first  = TransitionComputationStatus::e_success;
        transition_status.second = computed_transition_duration;
        if(computed_transition_duration > m_duration_lowerbound)
        {
            m_duration_lowerbound = computed_transition_duration;
            m_transition_constraint.set(GRB_DoubleAttr_RHS, -m_duration_lowerbound);
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }

        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    UpdateModelResult TraitsTransitionInfo::updateLowerBoundHeuristic(
            const std::shared_ptr<const Robot>& robot,
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
    {
        float max_distance = m_motion_planner_interface->computeMaxDistance(m_initial_configuration,
                                                                            m_terminal_configuration,
                                                                            problem_inputs);
        if (max_distance <= 0.0f)
        {
            return UpdateModelResult(std::shared_ptr<const TransitionFailure>(
                    new TransitionFailure{{.species                = robot->species()->name(),
                                                  .predecessor_task_index = m_predecessor_index,
                                                  .successor_task_index   = m_successor_index}}));
        }

        const float computed_transition_duration = max_distance / problem_inputs->getTraitsProblemInputs()->slowestSpeed();

        std::pair<TransitionComputationStatus, float>& transition_status = m_coalition[robot];
        transition_status.first  = TransitionComputationStatus::e_success;
        transition_status.second = computed_transition_duration;
        if(computed_transition_duration > m_duration_lowerbound)
        {
            m_duration_lowerbound = computed_transition_duration;
            return UpdateModelResult(UpdateModelResultType::e_updated);
        }

        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }
}  // namespace traits