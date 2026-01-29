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

// region Includes
// Global
#include <list>
#include <memory>
#include <tuple>
#include <unordered_map>
// External
#include <gurobi_c++.h>
// Local
#include <robin_hood/robin_hood.hpp>
#include "traits/common/utilities/hash_extension.hpp"
#include "traits/common/milp/milp_solver_base.hpp"
#include "traits/scheduling/scheduler_base.hpp"
#include "traits/scheduling/milp/schedule.hpp"

#include "traits/scheduling/milp/all_tasks_info.hpp"
#include "traits/scheduling/milp/all_transitions_info.hpp"
// endregion

namespace traits
{
    // Forward Declaration
    class MilpSchedulerParameters;
    class MutexIndicators;
    class TraitsSchedule;
    class ConfigurationBase;
    class Robot;

    /*!
     * A scheduling algorothm that uses a MILP formulation to solve a deterministic robot scheduling problem
     */
    class TraitsMilpScheduler
            : public TraitsSchedulerBase
            , public MilpSolverBase
    {
        public:
            // region Special Member Functions
            TraitsMilpScheduler() = delete;
            TraitsMilpScheduler(const TraitsMilpScheduler &) = delete;
            TraitsMilpScheduler(TraitsMilpScheduler &&) noexcept = default;
            ~TraitsMilpScheduler() = default;
            TraitsMilpScheduler &operator=(const TraitsMilpScheduler &) = delete;
            TraitsMilpScheduler &operator=(TraitsMilpScheduler &&) noexcept = default;
            // endregion

            /*!
             * \brief Constructor
             *
             * \param problem_inputs Inputs for a scheduling problem
             */
            explicit TraitsMilpScheduler(
                    const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
                    const std::shared_ptr<MutexIndicators>& mutex_indicators,
                    const std::shared_ptr<const NameScheme>& name_scheme,
                    const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& motion_planner_interface);

            explicit TraitsMilpScheduler(const std::shared_ptr<const TraitsSchedulerProblemInputs> &problem_inputs);

            //! \returns The number of times a MILP optimization was run
            [[nodiscard]] static unsigned int numIterations();

            inline void resetNumIterations();

        protected:
            //! \copydoc MilpSolverBase
            std::shared_ptr<const FailureReason> createObjective(GRBModel &model) final override;

            //! \copydoc MilpSolverBase
            UpdateModelResult updateModel(GRBModel &model) override;

            //! \copydoc SchedulerBase
            std::shared_ptr<const TraitsSchedulerResult> computeSchedule() override;

            //! \copydoc MilpSolverBase
            std::shared_ptr<const FailureReason> createVariables(GRBModel& model) override;

            //! \copydoc MilpSolverBase
            std::shared_ptr<const FailureReason> createConstraints(GRBModel& model) override;

            //! Builds a schedule from the MILP variables in \p model
            std::shared_ptr<const TraitsSchedule> createSchedule(GRBModel &model);

            //! \copydoc MilpSolverBase
            std::shared_ptr<const FailureReason> setupData() final override;

            /*!
             * \brief Add variables that are needed for the tasks (i.e. start and finish timepoints)
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the variables were successfully added
             */
            std::shared_ptr<const FailureReason> createTaskVariables(GRBModel& model);

            /*!
             * \brief Add variables that are needed for the task transitions (i.e. mutex indicators)
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the variables were successfully added
             */
            std::shared_ptr<const FailureReason> createTaskTransitionVariables(GRBModel& model);

            /*!
             * \brief Add variables that are needed for the objective function (i.e. makespan)
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the variables were successfully added
             */
            std::shared_ptr<const FailureReason> createObjectiveVariables(GRBModel& model);

            /*!
             * \brief Adds constraints that affect the tasks, but not task transitions (i.e. duration constraints)
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the constraints were successfully added
             */
            std::shared_ptr<const FailureReason> createTaskConstraints(GRBModel& model);

            /*!
             * \brief Adds constraints that affect task transitions
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the constraints were successfully added
             *
             * \note This also add TP precedence constraints even if there is not a robot transition associated with them
             */
            std::shared_ptr<const FailureReason> createTransitionConstraints(GRBModel& model);

            /*!
             * \brief Add constraints that affect the objective function
             *
             * \param model The model to add the constraints to
             *
             * \returns Whether the constraints were successfully added
             */
            std::shared_ptr<const FailureReason> createObjectiveConstraints(GRBModel& model);

            std::shared_ptr<const FailureReason> createDeadlineConstraints(GRBModel& model);


            /*!
             * \returns Big M
             * \see https://en.wikipedia.org/wiki/Big_M_method
             */
            [[nodiscard]] double getM() const;

            TraitsAllTasksInfo m_task_info;
            TraitsAllTransitionsInfo m_transition_info;
            GRBVar m_makespan;
            GRBVar m_objective_makespan;

            std::shared_ptr<const NameScheme> m_name_scheme;
            std::shared_ptr<const TraitsSchedulerMotionPlannerInterface> m_motion_planner_interface;

            static unsigned int s_num_iterations;
            //! These are the mutex constraint ids after the precedence constraints have been removed
            std::shared_ptr<MutexIndicators> m_mutex_indicators;

    };

    void TraitsMilpScheduler::resetNumIterations() {
        s_num_iterations = 0;
    }
}  // namespace ctaps