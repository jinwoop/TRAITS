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
#include "traits/scheduling/milp/milp_scheduler.hpp"

// region Includes
// External
#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/view/iota.hpp>
#include <magic_enum/magic_enum.hpp>
// Local
#include "traits/common/milp/milp_failure_reason.hpp"
#include "traits/common/milp/milp_solver_result.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/parameters/parameters_base.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/scheduler_motion_planner_interface.hpp"
#include "traits/scheduling/milp/name_scheme.hpp"
#include "traits/scheduling/milp/schedule.hpp"
#include "traits/scheduling/scheduler_result.hpp"
#include "traits/scheduling/milp/mutex_indicators.hpp"
#include "traits/scheduling/relative_deadline.hpp"
#include "traits/scheduling/absolute_deadline.hpp"
#include "traits/common/utilities/error.hpp"
// endregion

namespace traits
{
    unsigned int TraitsMilpScheduler::s_num_iterations = 0;

    TraitsMilpScheduler::TraitsMilpScheduler(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
                        const std::shared_ptr<MutexIndicators>& mutex_indicators,
                        const std::shared_ptr<const NameScheme>& name_scheme,
                        const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& motion_planner_interface)
        : TraitsSchedulerBase(problem_inputs)
        , MilpSolverBase(false)  // benders_decomposition = False
        , m_name_scheme(name_scheme)
        , m_mutex_indicators(mutex_indicators)
        , m_motion_planner_interface(motion_planner_interface)
        , m_task_info(problem_inputs, name_scheme, motion_planner_interface)
        , m_transition_info(m_task_info, problem_inputs, mutex_indicators, name_scheme, motion_planner_interface)
    {}

    TraitsMilpScheduler::TraitsMilpScheduler(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs)
            : TraitsMilpScheduler(problem_inputs,
                                std::make_shared<MutexIndicators>(problem_inputs, std::make_shared<NameScheme>(),
                                  (problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search ||
                                  problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_heuristic_makespan_lb)),
                                std::make_shared<NameScheme>(),
                                std::make_shared<TraitsSchedulerMotionPlannerInterface>())
    {}

    unsigned int TraitsMilpScheduler::numIterations()
    {
        return s_num_iterations;
    }

    std::shared_ptr<const traits::TraitsSchedulerResult> TraitsMilpScheduler::computeSchedule()
    {
        std::shared_ptr<MilpSolverResult> result = solveMilp(m_problem_inputs->schedulerParameters());
        if(result->failure())
        {
            return std::make_shared<TraitsSchedulerResult>(result->failureReason());
        }

        if (m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search)
        {
            s_num_iterations += result->numIterations();
        }

        if(auto schedule = createSchedule(*result->model()); schedule)
        {
            return std::make_shared<TraitsSchedulerResult>(schedule);
        }
        return std::make_shared<TraitsSchedulerResult>(std::make_shared<MilpFailureReason>());
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createVariables(GRBModel& model)
    {
        m_mutex_indicators->createVariables(model);

        if(std::shared_ptr<const FailureReason> failure_reason = createTaskVariables(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createTaskTransitionVariables(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createObjectiveVariables(model); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createConstraints(GRBModel& model)
    {
        if(std::shared_ptr<const FailureReason> failure_reason = createTaskConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createTransitionConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createDeadlineConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if(std::shared_ptr<const FailureReason> failure_reason = createObjectiveConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    double TraitsMilpScheduler::getM() const
    {
        return m_problem_inputs->scheduleWorstMakespan();
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::setupData()
    {
        // Tasks
        if(std::shared_ptr<const FailureReason> failure_reason = m_task_info.setupData(); failure_reason)
        {
            return failure_reason;
        }

        // Transitions
        if(std::shared_ptr<const FailureReason> failure_reason = m_transition_info.setupData(); failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createTaskVariables(GRBModel& model)
    {
        return m_task_info.createTaskVariables(model);
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createTaskTransitionVariables(GRBModel& model)
    {
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createObjectiveVariables(GRBModel& model)
    {
        // (-inf, inf) because NSQ return inf when MIP not feasible
        m_makespan =
                model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, m_name_scheme->createMakespanVariableName());

        m_objective_makespan=
                model.addVar(-GRB_INFINITY,
                             GRB_INFINITY,
                             0.0,
                             GRB_CONTINUOUS,
                             m_name_scheme->createObjectiveMakespanRatioName());
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createTaskConstraints(GRBModel& model)
    {
        if (std::shared_ptr<const FailureReason> failure_reason =
                m_task_info.createTaskLowerBoundConstraints(model); failure_reason)
        {
            return failure_reason;
        }

        if (std::shared_ptr<const FailureReason> failure_reason = m_task_info.createTaskUpperBoundConstraints(model);
                failure_reason)
        {
            return failure_reason;
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createTransitionConstraints(GRBModel& model)
    {
        if(m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search ||
           m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_heuristic_makespan_lb)
        {
            if(std::shared_ptr<const FailureReason> failure_reason =
                        m_transition_info.createPrecedenceTransitionConstraints(model);
                    failure_reason)
            {
                return failure_reason;
            }
            if(std::shared_ptr<const FailureReason> failure_reason =
                        m_transition_info.createMutexTransitionConstraints(model);
                    failure_reason)
            {
                return failure_reason;
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createDeadlineConstraints(GRBModel& model)
    {
        for (const std::shared_ptr<const DeadlineBase>& deadline : m_problem_inputs->getTraitsProblemInputs()->deadlines())
        {
            // Absolute Deadline Constraints
            if (deadline->deadlineType() == DeadlineType::e_absolute)
            {
                std::shared_ptr<const AbsoluteDeadline> abs_deadline =
                        std::dynamic_pointer_cast<const AbsoluteDeadline>(deadline);
                unsigned int task_nr = abs_deadline->timepoint().task();
                if (abs_deadline->timepoint().timepointType() == TimePointType::e_start)
                {
                    model.addConstr(m_task_info.taskStartTimePointVariable(task_nr) <= abs_deadline->bound(),
                                    m_name_scheme->createAbsoluteDeadlineConstraintStartName(task_nr));
                }
                else if (abs_deadline->timepoint().timepointType() == TimePointType::e_completion)
                {
                    model.addConstr(m_task_info.taskEndTimePointVariable(task_nr) <= abs_deadline->bound(),
                                    m_name_scheme->createAbsoluteDeadlineConstraintCompletionName(task_nr));
                }
                else
                {
                    throw createLogicError(fmt::format("Timepoint is neither start nor completion. Check the format."));
                }

            }
            // Relative Deadline Constraint
            else if (deadline->deadlineType() == DeadlineType::e_relative)
            {
                std::shared_ptr<const RelativeDeadline> rel_deadline =
                        std::dynamic_pointer_cast<const RelativeDeadline>(deadline);

                if (rel_deadline->predecessor().timepointType() == TimePointType::e_null ||
                    rel_deadline->successor().timepointType() == TimePointType::e_null)
                {
                    throw createLogicError(fmt::format("Timepoint is null. Check the format."));
                }

                unsigned int pred_nr = rel_deadline->predecessor().task();
                GRBVar& predecessor = rel_deadline->predecessor().timepointType() == TimePointType::e_start
                        ? m_task_info.taskStartTimePointVariable(pred_nr)
                        : m_task_info.taskEndTimePointVariable(pred_nr);

                unsigned int succ_nr = rel_deadline->successor().task();
                GRBVar& successor = rel_deadline->successor().timepointType() == TimePointType::e_start
                        ? m_task_info.taskStartTimePointVariable(succ_nr)
                        : m_task_info.taskEndTimePointVariable(succ_nr);

                model.addConstr(successor - predecessor <= rel_deadline->bound(),
                                m_name_scheme->createRelativeDeadlineConstraintName(pred_nr, succ_nr));
            }
            else
            {
                throw createLogicError(fmt::format("Deadline is neither Absolute nor Relative. Check the format."));
            }
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createObjectiveConstraints(GRBModel& model)
    {
        for(unsigned int task_nr: ::ranges::views::iota(0u, m_problem_inputs->numberOfPlanTasks()))
        {
            model.addConstr(
                    m_task_info.taskEndTimePointVariable(task_nr) - m_makespan <= 0,
                    m_name_scheme->createMakespanConstraintName(task_nr));
        }

        if (m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search) {
            model.addConstr((m_makespan - m_problem_inputs->scheduleBestMakespan()) /
                              (m_problem_inputs->scheduleWorstMakespan() - m_problem_inputs->scheduleBestMakespan())
                            <= m_objective_makespan,
                            m_name_scheme->createObjectiveMakespanRatioConstraintName());
        }

        return nullptr;
    }

    UpdateModelResult TraitsMilpScheduler::updateModel(GRBModel& model)
    {
        UpdateModelResult rv(UpdateModelResultType::e_no_update);

        const unsigned int num_robots = m_problem_inputs->numberOfRobots();
        std::vector<int> all_previous_tasks(num_robots, -1);

        std::vector<unsigned int> scheduled_order = m_task_info.scheduledOrder();
        for(unsigned int task_nr: scheduled_order)
        {
            // for every robot in a coalition
            for(const std::shared_ptr<const Robot>& robot: m_problem_inputs->coalition(task_nr))
            {
                auto iter                   = ::ranges::find_if(::ranges::views::iota(0u, num_robots),
                                                                [=, this](unsigned int r) -> bool
                                                                {
                                                                    return robot == m_problem_inputs->robot(r);
                                                                });
                const unsigned int robot_nr = *iter;
                const int previous_task_nr  = all_previous_tasks[robot_nr];

                UpdateModelResult update_model_result(UpdateModelResultType::e_no_update);

                // Initial Transition
                if(previous_task_nr == -1)
                {
                    update_model_result = m_task_info.updateTaskLowerBound(task_nr, robot);
                } else { // Other Transitions
                    update_model_result =
                            m_transition_info.updateTransitionDuration(model, previous_task_nr, task_nr, robot);
                }
                switch(update_model_result.type())
                {
                    case UpdateModelResultType::e_no_update:
                    {
                        // Do nothing
                        break;
                    }
                    case UpdateModelResultType::e_updated:
                    {
                        rv = update_model_result;
                        break;
                    }
                    case UpdateModelResultType::e_failure:
                    {
                        return update_model_result;
                    }
                }
                all_previous_tasks[robot_nr] = task_nr;
            }
        }
        return rv;
    }

    std::shared_ptr<const FailureReason> TraitsMilpScheduler::createObjective(GRBModel& model)
    {
        // Set all optimization to minimize (is the default, but we explicitly set anyway)
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        if(m_problem_inputs->schedulerParameters()->contains(constants::k_use_hierarchical_objective) &&
           m_problem_inputs->schedulerParameters()->get<bool>(constants::k_use_hierarchical_objective))
        {
            int k = 0;
            const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();

            // Note: lower priority objectives cannot degrade higher priority objectives

            if (m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search)
            {
                // Hierarchical objective (makespan and reward)
                model.setObjectiveN(GRBLinExpr(m_makespan), k++, 2); // minimize makespan

                for(int i = 0; i < num_tasks; ++i) {
                    model.setObjectiveN(GRBLinExpr(m_task_info.taskStartTimePointVariable(i)),
                                        k++,
                                        1);  //!< objective, index, priority
                }
            } else if (m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_heuristic_makespan_lb)
            {
                model.setObjectiveN(GRBLinExpr(m_makespan), k++, 1);
                for(int i = 0; i < num_tasks; ++i) {
                    model.setObjectiveN(GRBLinExpr(m_task_info.taskEndTimePointVariable(i)),
                                        k++,
                                        0);  //!< objective, index, priority
                }
            } else {
                throw createLogicError(fmt::format("How?"));
            }

        }
        return nullptr;
    }

    // MILP has solved the schedule for given tasks and robot allocation
    std::shared_ptr<const TraitsSchedule> TraitsMilpScheduler::createSchedule(GRBModel& model)
    {
        const double makespan                           = m_makespan.get(GRB_DoubleAttr_X);
        const double makespan_ratio                     = m_objective_makespan.get(GRB_DoubleAttr_X);

        std::vector<std::pair<float, float>> timepoints = m_task_info.timePoints();
        std::vector<std::pair<unsigned int, unsigned int>> precedence_set_mutex_constraints =
                m_mutex_indicators->precedenceSet();
        return std::make_shared<const TraitsSchedule>(makespan_ratio,
                                                    makespan,
                                                    timepoints,
                                                    precedence_set_mutex_constraints);
    }
}  // namespace traits
