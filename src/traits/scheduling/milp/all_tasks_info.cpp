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
#include "traits/scheduling/milp/all_tasks_info.hpp"

// External
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/scheduler_motion_planner_interface.hpp"
#include "traits/scheduling/milp/name_scheme.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/task.hpp"
#include "traits/common/utilities/error.hpp"

namespace traits
{
    TraitsAllTasksInfo::TraitsAllTasksInfo(
        const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
        const std::shared_ptr<const NameScheme>& name_scheme,
        const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& scheduler_motion_planner_interface)
        : m_problem_inputs(problem_inputs)
        , m_name_scheme(name_scheme)
        , m_scheduler_motion_planner_interface(scheduler_motion_planner_interface)
    {}

    std::shared_ptr<const FailureReason> TraitsAllTasksInfo::setupData()
    {
        const unsigned int num_tasks = m_problem_inputs->numberOfPlanTasks();
        m_task_infos.reserve(num_tasks);
        m_task_name_nr_map.reserve(num_tasks);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            // recomputes CoalitionView based on Allocation
            auto coalition = m_problem_inputs->coalition(task_nr);
            m_task_infos.emplace_back(coalition,
                                      task_nr,
                                      m_problem_inputs->planTask(task_nr),
                                      m_name_scheme,
                                      m_scheduler_motion_planner_interface,
                                      m_problem_inputs);
            if(std::shared_ptr<const FailureReason> failure_reason = m_task_infos.back().setupData(); failure_reason)
            {
                return failure_reason;
            }
            m_task_name_nr_map.emplace(m_problem_inputs->planTask(task_nr)->name(), task_nr);
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsAllTasksInfo::createTaskVariables(GRBModel& model)
    {
        for(TraitsTaskInfo& task_info: m_task_infos)
        {
            task_info.createTimePointVariables(model);
        }
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsAllTasksInfo::createTaskLowerBoundConstraints(GRBModel& model)
    {
        for (TraitsTaskInfo &task_info: m_task_infos) {
            task_info.createLowerBoundConstraint(model);
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitsAllTasksInfo::createTaskUpperBoundConstraints(GRBModel& model)
    {
        for(TraitsTaskInfo& task_info: m_task_infos)
        {
            if(m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search ||
               m_problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_heuristic_makespan_lb)
            {
                task_info.createUpperBoundConstraint(model);
                task_info.createTaskEndTimeConstraint(model);
            }
        }
        return nullptr;
    }

    UpdateModelResult TraitsAllTasksInfo::updateTaskLowerBound(unsigned int task_nr,
                                                            const std::shared_ptr<const Robot>& robot)
    {
        return m_task_infos[task_nr].updateLowerBound(robot);
    }

    UpdateModelResult TraitsAllTasksInfo::updateTaskLowerBoundHeuristic(unsigned int task_nr,
                                                                      const std::shared_ptr<const Robot>& robot)
    {
        return m_task_infos[task_nr].updateLowerBoundHeuristic(robot, m_problem_inputs);
    }

    UpdateModelResult TraitsAllTasksInfo::updateTaskUpperBound(unsigned int task_nr, float bound)
    {
        return m_task_infos[task_nr].updateUpperBound(bound);
    }

    std::vector<unsigned int> TraitsAllTasksInfo::scheduledOrder() const
    {
        std::vector<unsigned int> rv(m_task_infos.size());
        for(unsigned int i = 0, end = m_task_infos.size(); i < end; ++i)
        {
            rv[i] = i;
        }
        std::sort(rv.begin(),
                  rv.end(),
                  [this](unsigned int lhs, unsigned int rhs)
                  {
                      return m_task_infos[lhs].startTimePoint().get(GRB_DoubleAttr_X) <
                             m_task_infos[rhs].startTimePoint().get(GRB_DoubleAttr_X);
                  });
        return rv;
    }

    std::vector<std::pair<float, float>> TraitsAllTasksInfo::timePoints() const
    {
        std::vector<std::pair<float, float>> rv;
        const unsigned int num_tasks = m_task_infos.size();
        rv.reserve(num_tasks);
        for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            const float start   = m_task_infos[task_nr].startTimePoint().get(GRB_DoubleAttr_X);
            const float end     = m_task_infos[task_nr].endTimePoint().get(GRB_DoubleAttr_X);
            rv.push_back(std::pair(start, end));
        }
        return rv;
    }

    double TraitsAllTasksInfo::dualCut() const
    {
        double rv = 0.0;
        for(const TraitsTaskInfo& task: m_task_infos)
        {
            rv += task.dualCut();
        }
        return rv;
    }

}  // namespace traits