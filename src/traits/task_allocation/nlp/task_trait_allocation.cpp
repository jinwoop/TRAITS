/* Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Jinwoo Park
 *
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
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/robot.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"

namespace traits
{
    TaskTraitAllocation::TaskTraitAllocation(
            float trait_deficiencies,
            float trait_rate_deficiencies,
            std::vector<std::vector<std::vector<float>>>& robot_task_traits,
            std::vector<std::vector<std::vector<float>>>& robot_task_trait_rates,
            std::vector<std::vector<float>>& coalition_task_traits,
            std::vector<std::vector<float>>& coalition_task_trait_rates,
            std::vector<float> task_dynamic_durations,
            std::vector<float> task_total_durations,
            std::vector<float> task_intra_transition_durations,
            std::vector<std::vector<float>>& task_trait_durations,
            std::vector<float> result_robot_inter_task_transition_velocities,
            std::vector<float> result_robot_battery_consumptions)
            : m_result_trait_deficiencies(trait_deficiencies)
            , m_result_trait_rate_deficiencies(trait_rate_deficiencies)
            , m_robot_task_traits(robot_task_traits)
            , m_robot_task_trait_rates(robot_task_trait_rates)
            , m_coalition_task_traits(coalition_task_traits)
            , m_coalition_task_trait_rates(coalition_task_trait_rates)
            , m_task_dynamic_durations(task_dynamic_durations)
            , m_task_total_durations(task_total_durations)
            , m_task_intra_transition_durations(task_intra_transition_durations)
            , m_task_trait_durations(task_trait_durations)
            , m_result_robot_inter_task_transition_velocities(result_robot_inter_task_transition_velocities)
            , m_result_robot_battery_consumptions(result_robot_battery_consumptions)
    {}

    nlohmann::json TaskTraitAllocation::serializeToJson(
            const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
            const Eigen::MatrixXf& allocation) const
    {
        nlohmann::json solution_j;
        const Eigen::MatrixXf& allocation_matrix        =  allocation;

        // Collect and sort task information (name, id, time points, coalition, mp)
        {
            nlohmann::json task_list_j;
            for(unsigned int task_nr = 0, num_tasks = problem_inputs->numberOfPlanTasks(); task_nr < num_tasks;
                ++task_nr)
            {
                const std::shared_ptr<const Task>& task = problem_inputs->planTask(task_nr);
                nlohmann::json task_j;
                task_j["0_id"]               = task_nr;
                task_j["1_name"]             = task->name();

                task_j["2_task_dynamic_duration"] = m_task_dynamic_durations[task_nr];
                task_j["3_task_static_duration"] = problem_inputs->task(task_nr)->staticDuration();
                task_j["4_task_intra_transition"] = m_task_intra_transition_durations[task_nr];
                task_j["5_task_total_duration"] = m_task_total_durations[task_nr];
                task_j["6_coalition_traits"] = m_coalition_task_traits[task_nr];
                task_j["7_coalition_trait_rates"] = m_coalition_task_trait_rates[task_nr];
                task_j["8_robot_traits"] = m_robot_task_traits[task_nr];
                task_j["9_robot_trait_rates"] = m_robot_task_trait_rates[task_nr];

                task_list_j.push_back(task_j);
            }
            solution_j[constants::k_tasks] = task_list_j;
        }

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

                if (allocation.col(robot_nr).sum() < 0.5f) {
                    robot_j["inter_transition_velocity"]      = nullptr;
                    robot_j["battery_consumption"]            = nullptr;
                    robot_j["traits"] = nullptr;
                    robot_j["trait_rates"] = nullptr;
                    robot_list_j.push_back(robot_j);
                    continue;
                }

                robot_j["inter_transition_velocity"]      = m_result_robot_inter_task_transition_velocities[robot_nr];
                robot_j["battery_consumption"]            = m_result_robot_battery_consumptions[robot_nr];

                std::vector<std::vector<float>> traits;
                std::vector<std::vector<float>> trait_rates;
                traits.reserve(problem_inputs->numberOfRobots());
                trait_rates.reserve(problem_inputs->numberOfRobots());

                for (unsigned int task_nr = 0; task_nr < problem_inputs->numberOfTasks(); ++task_nr)
                {
                    traits.push_back(m_robot_task_traits[task_nr][robot_nr]);
                    trait_rates.push_back(m_robot_task_trait_rates[task_nr][robot_nr]);
                }

                robot_j["traits"] = traits;
                robot_j["trait_rates"] = trait_rates;

                robot_list_j.push_back(robot_j);
            }
            solution_j[constants::k_robots] = robot_list_j;
        }
        return solution_j;
    }
}  // namespace traits
