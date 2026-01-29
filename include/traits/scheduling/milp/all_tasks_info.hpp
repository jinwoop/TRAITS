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
#include <unordered_map>
// External
#include "gurobi_c++.h"
// Local
#include "traits/scheduling/milp/task_info.hpp"

namespace traits
{
    // Forward Declarations
    class TraitsSchedulerProblemInputs;
    class TraitsSchedulerMotionPlannerInterface;
    class NameScheme;
    class FailureReason;
    class MutexIndicators;

    /*!
     * Contains info about all tasks needed for DeterministicMilpSchedulerBase
     *
     * \see DeterministicMilpSchedulerBase
     */
    class TraitsAllTasksInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param problem_inputs The inputs to the scheduling problem
         * \param name_scheme The scheme for naming variables and constraints
         * \param scheduler_motion_planner_interface An interface between the scheduler and motion planner
         */
        explicit TraitsAllTasksInfo(
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<const NameScheme>& name_scheme,
            const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& scheduler_motion_planner_interface);

        /*!
         * Sets up the data needed to create variables/constraints
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> setupData();

        /*!
         * Adds task variables to \p model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model);

        /*!
         * Adds constraints on the lower bound for the start times of tasks to the model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createTaskLowerBoundConstraints(GRBModel& model);

        /*!
         * Adds constraints on the upper bound for the start times of tasks to the model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createTaskUpperBoundConstraints(GRBModel& model);

        /*!
         * Tries to update the lower bound of task \p task_nr's timepoints
         *
         * \param task_nr The index of the task
         * \param robot The robot to compute a motion plan for
         *
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        UpdateModelResult updateTaskLowerBound(unsigned int task_nr, const std::shared_ptr<const Robot>& robot);

        UpdateModelResult updateTaskLowerBoundHeuristic(unsigned int task_nr,
                                                        const std::shared_ptr<const Robot>& robot);

        /*!
         * Tries to update the upper bound of task \p task_nr's timepoints
         *
         * \param task_nr The index of the task
         * \param previous_task_nr The index of the previous task
         *
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        UpdateModelResult updateTaskUpperBound(unsigned int task_nr, float bound);

        //! \returns The MILP variable representing the start of task \p task_nr
        [[nodiscard]] inline GRBVar& taskStartTimePointVariable(unsigned int task_nr);

        [[nodiscard]] inline GRBVar& taskEndTimePointVariable(unsigned int task_nr);

        //! \returns A list of task indices in the order with which they start in the schedule
        [[nodiscard]] std::vector<unsigned int> scheduledOrder() const;

        //! \returns A list of timepoints for the start and finish of each of the tasks
        std::vector<std::pair<float, float>> timePoints() const;

        //! \returns The part of the optimality cut related to the tasks
        [[nodiscard]] double dualCut() const;

        //! \returns The duration of a specific task
        [[nodiscard]] inline float taskDuration(unsigned int task_nr) const;

        //! \returns The lower bound of a specific task
        [[nodiscard]] inline float taskLowerBound(unsigned int task_nr) const;

        //! \returns The upper bound of a specific task
        [[nodiscard]] inline float taskUpperBound(unsigned int task_nr) const;

       private:
        std::shared_ptr<const TraitsSchedulerProblemInputs> m_problem_inputs;
        std::shared_ptr<const TraitsSchedulerMotionPlannerInterface> m_scheduler_motion_planner_interface;
        std::shared_ptr<const NameScheme> m_name_scheme;
        std::vector<TraitsTaskInfo> m_task_infos;
        std::unordered_map<std::string, unsigned int> m_task_name_nr_map;
    };

    // Inline Functions
    GRBVar& TraitsAllTasksInfo::taskStartTimePointVariable(unsigned int task_nr)
    {
        return m_task_infos[task_nr].startTimePoint();
    }

    GRBVar& TraitsAllTasksInfo::taskEndTimePointVariable(unsigned int task_nr)
    {
        return m_task_infos[task_nr].endTimePoint();
    }

    float TraitsAllTasksInfo::taskDuration(unsigned int task_nr) const
    {
        return m_task_infos[task_nr].duration();
    }

    float TraitsAllTasksInfo::taskLowerBound(unsigned int task_nr) const
    {
        return m_task_infos[task_nr].lowerBound();
    }

    float TraitsAllTasksInfo::taskUpperBound(unsigned int task_nr) const
    {
        return m_task_infos[task_nr].upperBound();
    }

}  // namespace traits