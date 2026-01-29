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
#pragma once

// Global
#include <memory>
#include <vector>
// Local

namespace traits
{
    // Forward Declarations
    class Task;
    class Robot;
    class Species;
    class ConfigurationBase;
    class TraitsSchedulerProblemInputs;

    /*!
     * The standard interface between scheduling and motion planning
     */
    class TraitsSchedulerMotionPlannerInterface
    {
        public:
            //! \returns How long \p coalition will take to accomplish \p task_nr'th task
            [[nodiscard]] float computeTaskDuration(
                    const std::shared_ptr<const Task>& task,
                    const std::vector<std::shared_ptr<const Robot>>& coalition) const;

            [[nodiscard]] float computeTaskDuration(
                    const std::shared_ptr<const Task>& task,
                    const std::vector<std::shared_ptr<const Robot>>& coalition,
                    const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const;

            /*!
             * \returns Whether the motion plan from a \p robot's initial configuration to a specific \p configuration has
             * already been computed
             */
            [[nodiscard]] bool isInitialTransitionMemoized(
                    const std::shared_ptr<const ConfigurationBase>& configuration,
                    const std::shared_ptr<const Robot>& robot) const;

            /*!
             * \returns The time it will take a \p robot to transition from it initial configuration to \p configuration
             */
            [[nodiscard]] float computeInitialTransitionDuration(
                    const std::shared_ptr<const ConfigurationBase>& configuration,
                    const std::shared_ptr<const Robot>& robot) const;

            /*!
             * \returns An estimate of the time it will take a \p robot to transition from it initial configuration to \p
             * configuration
             */
            [[nodiscard]] float computeInitialTransitionDurationHeuristic(
                    const std::shared_ptr<const ConfigurationBase>& configuration,
                    const std::shared_ptr<const Robot>& robot) const;

            /*!
             * \returns Whether the motion plan from \p initial to \p goal has already been computed for \p robot
             */
            [[nodiscard]] bool isTransitionMemoized(const std::shared_ptr<const ConfigurationBase>& initial,
                                                    const std::shared_ptr<const ConfigurationBase>& goal,
                                                    const std::shared_ptr<const Robot>& robot) const;

            /*!
             * \returns The time it will take a \p robot to transition from \p initial to \p goal
             */
            [[nodiscard]] float computeTransitionDuration(const std::shared_ptr<const ConfigurationBase>& initial,
                                                          const std::shared_ptr<const ConfigurationBase>& goal,
                                                          const std::shared_ptr<const Robot>& robot) const;

            /*!
             * \returns An estimate of the time it will take a \p robot to transition from \p initial to \p goal
             */
            [[nodiscard]] float computeTransitionDurationHeuristic(
                    const std::shared_ptr<const ConfigurationBase>& initial,
                    const std::shared_ptr<const ConfigurationBase>& goal,
                    const std::shared_ptr<const Robot>& robot) const;

            [[nodiscard]] float computeDuration(
                    const std::shared_ptr<const ConfigurationBase>& initial,
                    const std::shared_ptr<const ConfigurationBase>& goal,
                    const std::shared_ptr<const Species>& species,
                    const float velocity) const;

            [[nodiscard]] float computeDistance(
                const std::shared_ptr<const ConfigurationBase>& initial,
                const std::shared_ptr<const ConfigurationBase>& goal,
                const std::shared_ptr<const Species>& species) const;

            [[nodiscard]] float computeMaxDistance(
                    const std::shared_ptr<const ConfigurationBase>& initial,
                    const std::shared_ptr<const ConfigurationBase>& goal,
                    const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const;

    };

}  // namespace traits