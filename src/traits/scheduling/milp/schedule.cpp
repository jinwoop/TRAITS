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
#include "traits/scheduling/milp/schedule.hpp"

#include <iostream>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"

namespace traits
{
    TraitsSchedule::TraitsSchedule(
        const float makespan_ratio,
        const float makespan,
        const std::vector<std::pair<float, float>>& time_points,
        const std::vector<std::pair<unsigned int, unsigned int>>& precedence_set_mutex_constraints)
        : m_makespan_ratio(makespan_ratio)
        , m_makespan(makespan)
        , m_time_points(time_points)
        , m_precedence_set_mutex_constraints(precedence_set_mutex_constraints)
    {}

    nlohmann::json TraitsSchedule::serializeToJson(
        const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const
    {
        nlohmann::json solution_j;
        const Eigen::MatrixXf& allocation_matrix        = problem_inputs->allocation();
        solution_j[constants::k_allocation]             = allocation_matrix;
        float makespan_doublecheck = 0.0f;
        solution_j[constants::k_precedence_constraints] = problem_inputs->precedenceConstraints();
        solution_j[constants::k_precedence_set_mutex_constraints] = m_precedence_set_mutex_constraints;


        std::vector<std::vector<unsigned int>> individual_robot_plans(allocation_matrix.cols());

        // Collect and sort task information (name, id, time points, coalition, mp)
        {
            nlohmann::json task_list_j;
            for(unsigned int task_nr = 0, num_tasks = problem_inputs->numberOfPlanTasks(); task_nr < num_tasks;
                ++task_nr)
            {
                const std::shared_ptr<const Task>& task = problem_inputs->planTask(task_nr);
                nlohmann::json task_j;
                task_j[constants::k_name]             = task->name();
                task_j[constants::k_id]               = task_nr;
                task_j[constants::k_start_timepoint]  = taskStart(task_nr);
                task_j[constants::k_finish_timepoint] = taskEnd(task_nr);
                makespan_doublecheck = std::max(makespan_doublecheck, taskEnd(task_nr));
                std::vector<unsigned int> coalition_ids;
                std::vector<std::shared_ptr<const Robot>> coalition;
                for(unsigned int robot_nr = 0, num_robots = problem_inputs->numberOfRobots(); robot_nr < num_robots;
                    ++robot_nr)
                {
                    if(allocation_matrix(task_nr, robot_nr))
                    {
                        coalition_ids.push_back(robot_nr);
                        coalition.push_back(problem_inputs->robot(robot_nr));
                        individual_robot_plans[robot_nr].push_back(task_nr);
                    }
                }
                task_j[constants::k_coalition] = coalition_ids;
                // return memoized motion plans
                std::shared_ptr<const MotionPlannerQueryResultBase> motion_planning_result =
                    task->motionPlanningQuery(coalition);
                task_j[constants::k_execution_motion_plan] = motion_planning_result;
                task_list_j.push_back(task_j);
            }
            solution_j[constants::k_tasks] = task_list_j;
        }
        solution_j[constants::k_makespan]               = m_makespan;

        // Collect and store robot plan information (name, id, individual_plan, transitions)
        {
            nlohmann::json robot_list_j;

            for(unsigned int robot_nr = 0, num_robots = problem_inputs->numberOfRobots(); robot_nr < num_robots;
                ++robot_nr)
            {
                nlohmann::json robot_j;
                const std::shared_ptr<const Robot>& robot = problem_inputs->robot(robot_nr);
                robot_j[constants::k_name]                = robot->name();  //!< name
                robot_j[constants::k_id]                  = robot_nr;       //!< id

                const auto& time_points = m_time_points;
                std::sort(individual_robot_plans[robot_nr].begin(),
                          individual_robot_plans[robot_nr].end(),
                          [&time_points](const unsigned int& lhs, const unsigned int& rhs)
                          {
                              return time_points[lhs].first < time_points[rhs].first;
                          });

                robot_j[constants::k_individual_plan] = individual_robot_plans[robot_nr];  //!< individual plan

                nlohmann::json transition_list_j;
                if(!individual_robot_plans[robot_nr].empty())
                {
                    // Initial transition
                    std::shared_ptr<const Task> task = problem_inputs->planTask(individual_robot_plans[robot_nr][0]);
                    transition_list_j.push_back(robot->motionPlanningQuery(task->initialConfiguration()));

                    std::shared_ptr<const Task> previous_task = task;  // was using & (reference, so memory location kept changing; i.e., task = prev_task)

                    for(unsigned int i = 1, end = individual_robot_plans[robot_nr].size(); i < end; ++i)
                    {
                        task = problem_inputs->planTask(individual_robot_plans[robot_nr][i]);
                        transition_list_j.push_back(robot->motionPlanningQuery(previous_task->terminalConfiguration(),
                                                                               task->initialConfiguration()));
                        previous_task = task;
                    }
                }
                robot_j[constants::k_transitions] = transition_list_j;  //!< transitions

                robot_list_j.push_back(robot_j);
            }
            solution_j[constants::k_robots] = robot_list_j;
        }
        return solution_j;
    }
}  // namespace traits