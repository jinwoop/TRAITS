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
#include "traits/scheduling/scheduler_motion_planner_interface.hpp"

#include "fmt/format.h"

// Local
#include "traits/geometric_planning/configurations/configuration_base.hpp"
#include "traits/geometric_planning/query_results/ompl_motion_planner_query_result.hpp"
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/task.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/scheduler_objective_enum.hpp"

namespace traits
{
    float TraitsSchedulerMotionPlannerInterface::computeTaskDuration(
        const std::shared_ptr<const Task>& task,
        const std::vector<std::shared_ptr<const Robot>>& coalition) const
    {
        return task->computeDuration(coalition);
    }

    float TraitsSchedulerMotionPlannerInterface::computeTaskDuration(
            const std::shared_ptr<const Task>& task,
            const std::vector<std::shared_ptr<const Robot>>& coalition,
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const
    {
        if (problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_search ||
            problem_inputs->schedulerObjectiveType() == SchedulerObjectiveType::e_heuristic_makespan_lb) {
            return task->computeDuration(coalition);
        }

        float distance = computeMaxDistance(task->initialConfiguration(), task->terminalConfiguration(), problem_inputs);

        return distance / problem_inputs->getTraitsProblemInputs()->slowestSpeed() + task->staticDuration() +
                task->maximum_dynamic_task_duration();
    }

    bool TraitsSchedulerMotionPlannerInterface::isInitialTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        return robot->isMemoized(configuration);
    }

    float TraitsSchedulerMotionPlannerInterface::computeInitialTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& configuration,
        const std::shared_ptr<const Robot>& robot) const
    {
        return robot->durationQuery(configuration);
    }

    bool TraitsSchedulerMotionPlannerInterface::isTransitionMemoized(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& goal,
        const std::shared_ptr<const Robot>& robot) const
    {
        return robot->isMemoized(initial, goal);
    }

    float TraitsSchedulerMotionPlannerInterface::computeTransitionDuration(
        const std::shared_ptr<const ConfigurationBase>& initial,
        const std::shared_ptr<const ConfigurationBase>& goal,
        const std::shared_ptr<const Robot>& robot) const
    {
        return robot->durationQuery(initial, goal);
    }

    float TraitsSchedulerMotionPlannerInterface::computeInitialTransitionDurationHeuristic(
            const std::shared_ptr<const ConfigurationBase>& configuration,
            const std::shared_ptr<const Robot>& robot) const
    {
        return robot->initialConfiguration()->euclideanDistance(*configuration) / robot->max_speed();
    }

    float TraitsSchedulerMotionPlannerInterface::computeTransitionDurationHeuristic(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& goal,
            const std::shared_ptr<const Robot>& robot) const
    {
        return initial->euclideanDistance(*goal) / robot->max_speed();
    }

    float TraitsSchedulerMotionPlannerInterface::computeDuration(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& goal,
            const std::shared_ptr<const Species>& species,
            const float velocity) const
    {
        auto result = species->motionPlanner()->query(species, initial, goal);
        return std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->length() / velocity;
    }

    float TraitsSchedulerMotionPlannerInterface::computeDistance(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& goal,
            const std::shared_ptr<const Species>& species) const
    {
       auto result = species->motionPlanner()->query(species, initial, goal);
       return std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->length();
    }

    float TraitsSchedulerMotionPlannerInterface::computeMaxDistance(
            const std::shared_ptr<const ConfigurationBase>& initial,
            const std::shared_ptr<const ConfigurationBase>& goal,
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const
    {
        float max_distance = -1.0f;
        for (const auto& species : problem_inputs->getTraitsProblemInputs()->multipleSpecies()) {
            auto result = species->motionPlanner()->query(species, initial, goal);
            max_distance = std::max(std::dynamic_pointer_cast<const OmplMotionPlannerQueryResult>(result)->length(),
                                    max_distance);
        }
        return max_distance;
    }
}  // namespace traits