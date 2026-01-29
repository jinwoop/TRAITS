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
#include "traits/task.hpp"

// Local
#include "traits/geometric_planning/motion_planning_enums.hpp"
#include "traits/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/task_planning/sas/sas_action.hpp"
#include "traits/geometric_planning/configurations/configuration_base.hpp"

namespace traits
{
    Task::Task(const std::shared_ptr<SasAction>& symbolic_action,
               const Eigen::VectorXf& desired_traits,
               const std::shared_ptr<const ConfigurationBase>& initial_configuration,
               const std::shared_ptr<const ConfigurationBase>& terminal_configuration)
           : Task(symbolic_action,
                      desired_traits,
                      Eigen::VectorXf(desired_traits).setZero(),
                      Eigen::VectorXf(desired_traits).setOnes(),
                      initial_configuration,
                      terminal_configuration,
                      0.0f,
                      0.0f,
                      0.0f,
                      0.0f) // this would lead to SCHEDULER error because incorrect UB causes TETAQ > 1.0f
    {}

    Task::Task(const std::shared_ptr<SasAction>& symbolic_action,
               const Eigen::VectorXf& desired_traits,
               const Eigen::VectorXf& minimum_trait_rates,
               const Eigen::VectorXf& traits_aggregatable,
               const std::shared_ptr<const ConfigurationBase>& initial_configuration,
               const std::shared_ptr<const ConfigurationBase>& terminal_configuration,
               float fastest_species_speed,
               float slowest_species_speed,
               float total_task_duration,
               float maximum_dynamic_task_duration)
        : m_symbolic_action(symbolic_action)
        , m_desired_traits(desired_traits)
        , m_minimum_trait_rates(minimum_trait_rates)
        , m_traits_aggregatable(traits_aggregatable)
        , m_initial_configuration(initial_configuration)
        , m_terminal_configuration(terminal_configuration)
        , m_fastest_species_speed(fastest_species_speed)
        , m_slowest_species_speed(slowest_species_speed)
        , m_total_task_duration(total_task_duration)
        , m_maximum_dynamic_task_duration(maximum_dynamic_task_duration)
    { }

    unsigned int Task::id() const
    {
        return m_symbolic_action->id();
    }

    const std::string& Task::name() const
    {
        return m_symbolic_action->name();
    }

    float Task::staticDuration() const
    {
        return m_symbolic_action->duration();
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> Task::motionPlanningQuery(
        const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        if(coalition.empty())
        {
            return nullptr;
        }

        std::shared_ptr<const Robot> widest_robot = nullptr;
        for(const std::shared_ptr<const Robot>& robot: coalition)
        {
            if(widest_robot == nullptr || robot->boundingRadius() > widest_robot->boundingRadius())
            {
                widest_robot = robot;
            }
        }
        return widest_robot->motionPlanningQuery(m_initial_configuration, m_terminal_configuration);
    }

    float Task::computeDurationLBHeuristic() const
    {
        return m_initial_configuration->euclideanDistance(*m_terminal_configuration) / m_fastest_species_speed
               + m_symbolic_action->duration();
    }

    float Task::computeDuration(const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        if(coalition.empty())
        {
            if (m_fastest_species_speed > 0.0f)
            {
                return computeDurationLBHeuristic();
            }
            return m_symbolic_action->duration();   // this is lower bound
        }

        // Get the route for the widest robot
        const std::shared_ptr<const MotionPlannerQueryResultBase> motion_planning_result =
            motionPlanningQuery(coalition);

        if(motion_planning_result == nullptr || motion_planning_result->status() != MotionPlannerQueryStatus::e_success)
        {
            return -1.0f;
        }

        return m_total_task_duration;
    }
}  // namespace traits