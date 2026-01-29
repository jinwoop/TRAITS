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
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// External
#include <gurobi_c++.h>
#include <range/v3/view/transform.hpp>
// Local
#include "traits/common/utilities/hash_extension.hpp"
#include "traits/common/utilities/update_model_result.hpp"
#include "traits/scheduling/milp/all_tasks_info.hpp"
#include "traits/scheduling/milp/transition_info.hpp"

namespace traits
{
    // Forward Declarations
    class NameScheme;
    class FailureReason;
    class MutexIndicators;
    class Robot;
    class TraitsSchedulerMotionPlannerInterface;
    class TraitsSchedulerProblemInputs;

    /*!
     *
     */
    class TraitsAllTransitionsInfo
    {
       public:
        /*!
         * Constructor
         *
         * \param problem_inputs The inputs to the scheduling problem
         * \param mutex_indicators
         * \param name_scheme The scheme for naming variables and constraints
         * \param scheduler_motion_planner_interface An interface between the scheduler and motion planner
         */
        explicit TraitsAllTransitionsInfo(
            TraitsAllTasksInfo& tasks_info,
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
            const std::shared_ptr<MutexIndicators>& mutex_indicators,
            const std::shared_ptr<const NameScheme>& name_scheme,
            const std::shared_ptr<const TraitsSchedulerMotionPlannerInterface>& scheduler_motion_planner_interface);

        /*!
         * Sets up the data needed to create variables/constraints
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> setupData();

        /*!
         * Adds constraints on precedence transition durations to \p model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createPrecedenceTransitionConstraints(GRBModel& model);

        /*!
         * Adds constraints on mutex transition durations to \p model
         *
         * \param model The MILP model
         *
         * \returns A reason for failure if it fails
         */
        std::shared_ptr<const FailureReason> createMutexTransitionConstraints(GRBModel& model);

        /*!
         * Attempt to update the model by updating the transition duration through an mp query
         *
         * \param model The model
         * \param first A task index
         * \param second Another task index
         * \param robot The robot for the mp query
         *
         * \returns The results of the attempt
         */
        [[nodiscard]] UpdateModelResult updateTransitionDuration(GRBModel& model,
                                                                 unsigned int first,
                                                                 unsigned int second,
                                                                 const std::shared_ptr<const Robot>& robot);

        [[nodiscard]] UpdateModelResult updateTransitionDurationHeuristic(GRBModel& model,
                                                                          unsigned int first,
                                                                          unsigned int second,
                                                                          const std::shared_ptr<const Robot>& robot);

        /*!
         * \brief Computes the components of the optimality related to the transitions
         *
         * \f[
         *      \sum_{(i, j) \in \mathcal{P}} (d_i^q + x_{ij}^q) \beta_{ij}^q +
         *      \sum_{(i, j) \in \mathcal{M}} (d_i^q + x_{ij}^q - A_{ij}^q) \gamma_{ij}^q +
         *      \sum_{(i, j) \in \mathcal{M}} (d_j^q + x_{ji}^q - B_{ij}^q) \delta_{ij}^q
         * \f]
         *
         * \f[
         *     A_{ij} = M (1 - p_{ij})
         *     B_{ij} = M p_{ij}
         * \f]
         *
         * \tparam ReturnType
         * \tparam Variable
         *
         * \param master_mutex_indicators The mutex indicators from the master problem
         *
         * \returns The components of the optimality cut related to the transitions
         */
        template <DualCutReturnType ReturnType, DualCutVariableType Variable>
        [[nodiscard]] ReturnType dualCut(
            std::unordered_map<std::pair<unsigned int, unsigned int>, Variable>& master_mutex_indicators) const
        {
            ReturnType rv = dualCutBetaComponent();

            const double M = getM();
            for(auto [first, second, var]:
                master_mutex_indicators | ranges::views::transform(
                                              [](const auto& iter)
                                              {
                                                  return std::tuple(iter.first.first, iter.first.second, iter.second);
                                              }))
            {
                // first -> second
                {
                    const double predecessor_task_duration = m_tasks_info.taskDuration(first);
                    rv += m_transition_infos[first][second]->template dualCut<ReturnType>(predecessor_task_duration,
                                                                                          M * (1.0 - var));
                }

                // second -> first
                {
                    const double predecessor_task_duration = m_tasks_info.taskDuration(second);
                    ReturnType term =
                        m_transition_infos[second][first]->template dualCut<ReturnType>(predecessor_task_duration,
                                                                                        M * var);
                    rv += term;
                }
            }
            return rv;
        }

        //! \returns The minimum transition duration from \p i to \p j
        [[nodiscard]] inline float transitionDurationLowerBound(unsigned int i, unsigned int j) const;

       private:
        //! \returns The beta component (precedence constraints) of the optimality cut
        [[nodiscard]] double dualCutBetaComponent() const;

        //! \returns M
        [[nodiscard]] double getM() const;

        GRBVar* task_end_upper_bounds;
        std::vector<GRBVar*> aux_vars_quadexpr;

        TraitsAllTasksInfo& m_tasks_info;  //!< Needed to get timepoint variables
        std::vector<std::vector<std::optional<TraitsTransitionInfo>>> m_transition_infos;
        std::shared_ptr<MutexIndicators> m_mutex_indicators;

        std::shared_ptr<const TraitsSchedulerProblemInputs> m_problem_inputs;
        std::shared_ptr<const NameScheme> m_name_scheme;
        std::shared_ptr<const TraitsSchedulerMotionPlannerInterface> m_motion_planner_interface;
    };

    // Inline Functions
    float TraitsAllTransitionsInfo::transitionDurationLowerBound(unsigned int i, unsigned int j) const
    {
        return m_transition_infos[i][j].value().durationLowerBound();
    }
}  // namespace traits