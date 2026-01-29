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
#include <gurobi_c++.h>
// Local
#include "traits/common/utilities/custom_views.hpp"
#include "traits/common/utilities/update_model_result.hpp"
#include "traits/scheduling/milp/deterministic/transition_computation_status.hpp"

namespace traits
{
    // Forward Declarations
    class Robot;
    class Task;
    class NameScheme;
    class TraitsSchedulerProblemInputs;
    class TraitsSchedulerMotionPlannerInterface;
    class FailureReason;

    /*!
     * Creates all the information for building MILP model components representing this task
     */
    class TraitsTaskInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param coalition The coalition of robots assigned to this task
         * \param plan_task_nr The index for this task in the plan
         * \param task The task
         * \param name_scheme The scheme for naming variables/constraints
         * \param motion_planner_interface The interface to the motion planner
         */
        TraitsTaskInfo(TraitsCoalitionView coalition,
                    unsigned int plan_task_nr,
                    const std::shared_ptr<const Task>& task,
                    const std::shared_ptr<const NameScheme>& name_scheme,
                    const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& motion_planner_interface,
                    const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs);

        /*!
         * Sets up the data for calculating constraints and bounds
         *
         * \returns The reason for failure if there is one
         */
        [[nodiscard]] std::shared_ptr<const FailureReason> setupData();

        //! Creates the start and finish timepoint variables and adds them to the model
        void createTimePointVariables(GRBModel& model);

        //! Creates a redundant lowerbound constraint in order to get the dual value
        void createLowerBoundConstraint(GRBModel& model);

        //! Creates a upperbound constraint in order meet absolute deadline
        void createUpperBoundConstraint(GRBModel& model);

        void createTaskEndTimeConstraint(GRBModel& model);
        void createTaskStartUpperBoundConstraint(GRBModel& model);

        /*!
         * Tries to update the lower bound of this task's timepoints
         *
         * \param robot The robot to compute a motion plan for
         * \returns Whether the model was updated or not (or a failure occurred)
         */
        [[nodiscard]] UpdateModelResult updateLowerBound(const std::shared_ptr<const Robot>& robot);

        [[nodiscard]] UpdateModelResult updateLowerBoundHeuristic(
                const std::shared_ptr<const Robot>& robot,
                const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs);

        UpdateModelResult updateLowerBoundHeuristicForce(float lower_bound);
        UpdateModelResult updateLowerBoundHeuristicMax(float lower_bound);

        [[nodiscard]] UpdateModelResult updateUpperBound(float bound);


        //! \returns The task object
        [[nodiscard]] inline std::shared_ptr<const Task>& task();

        //! \returns The duration of the task
        [[nodiscard]] inline float duration() const;

        /*!
         * \returns The lowerbound for the start of the task
         *
         * \note Not all motion plans may have been calculated, so this could be based on heuristics
         */
        [[nodiscard]] inline float lowerBound() const;

        /*!
         * \returns The upperbound for the end of the task
         */
        [[nodiscard]] inline float upperBound() const;

        //! \returns The start timepoint
        [[nodiscard]] inline GRBVar& startTimePoint();

        //! \returns The start timepoint
        [[maybe_unused]] [[nodiscard]] inline const GRBVar& startTimePoint() const;

        //! \returns The end timepoint
        [[nodiscard]] inline GRBVar& endTimePoint();

        //! \returns The end timepoint
        [[maybe_unused]] [[nodiscard]] inline const GRBVar& endTimePoint() const;

        //! \returns The list of robots assigned to this task
        [[nodiscard]] std::vector<std::shared_ptr<const Robot>> coalition() const;

        /*!
         * \f[
         *      \sum_{i \in I}  x_i^q \epsilon_i^q
         * \f]
         *
         * \returns The component of the optimality cut for this task
         */
        [[nodiscard]] double dualCut() const;

       protected:
        float m_duration;
        float m_lower_bound;
        float m_upper_bound;
        unsigned int m_plan_task_nr;
        std::shared_ptr<const Task> m_task;
        std::unordered_map<std::shared_ptr<const Robot>, std::pair<TransitionComputationStatus, float>> m_coalition;
        GRBVar m_start_time_point;
        GRBVar m_end_time_point;
        GRBConstr m_lower_bound_constraint;
        GRBConstr m_upper_bound_constraint;
        GRBConstr m_task_end_time_constraint;

        std::shared_ptr<const NameScheme> m_name_scheme;
        std::shared_ptr<const TraitsSchedulerMotionPlannerInterface> m_motion_planner_interface;
        std::shared_ptr<const TraitsSchedulerProblemInputs> m_problem_inputs;
    };

    // Inline Functions
    std::shared_ptr<const Task>& TraitsTaskInfo::task()
    {
        return m_task;
    }

    float TraitsTaskInfo::duration() const
    {
        return m_duration;
    }

    float TraitsTaskInfo::lowerBound() const
    {
        return m_lower_bound;
    }

    float TraitsTaskInfo::upperBound() const
    {
        return m_upper_bound;
    }

    GRBVar& TraitsTaskInfo::startTimePoint()
    {
        return m_start_time_point;
    }

    [[maybe_unused]] const GRBVar& TraitsTaskInfo::startTimePoint() const
    {
        return m_start_time_point;
    }

    GRBVar& TraitsTaskInfo::endTimePoint()
    {
        return m_end_time_point;
    }

    [[maybe_unused]] const GRBVar& TraitsTaskInfo::endTimePoint() const
    {
        return m_end_time_point;
    }

}  // namespace traits