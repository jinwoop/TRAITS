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
#include <set>
#include <vector>
// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/problem_inputs/problem_inputs.hpp"
#include "traits/scheduling/scheduler_objective_enum.hpp"

namespace traits
{
    /*!
     * \brief Container for the inputs to a scheduling problem
     */
    class TraitsSchedulerProblemInputs : public ProblemInputs
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param allocation
         */
        explicit TraitsSchedulerProblemInputs(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                                             const Eigen::MatrixXf& allocation,
                                             SchedulerObjectiveType scheduler_objective_type
                                                = SchedulerObjectiveType::e_search);

        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param allocation
         */
        explicit TraitsSchedulerProblemInputs(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
                                             Eigen::MatrixXf&& allocation,
                                             SchedulerObjectiveType scheduler_objective_type
                                              = SchedulerObjectiveType::e_search);

        /*!
         * Throws an exception if the output from task planning isn't valid
         *
         * Reasons it could be invalid:
         * - mutex constraint uses a index that is out of range of the number of plan tasks
         * - precedence constraint uses a index that is out of range of the number of plan tasks
         */
        void validate() const;

        /*!
         * \returns A view of the coalition of robots assigned to task \p task_nr
         *
         * \note The typename for the view is unwieldy so we use auto. This requires the implementation of the function
         *       to be in the header
         */
        [[nodiscard]] TraitsCoalitionView coalition(unsigned int task_nr) const;

        /*!
         * \returns A view of the coalition of robots assigned to both tasks \p i and \p j
         *
         * \note The typename for the view is unwieldy so we use auto. This requires the implementation of the function
         *       to be in the header
         */
        [[nodiscard]] TraitsCoalitionView transitionCoalition(unsigned int i, unsigned int j) const;

        // TraitsProblemInputs is getting TraitsProblemInputs object
        [[nodiscard]] inline const std::shared_ptr<const TraitsProblemInputs>& getTraitsProblemInputs() const;

        // Output from Task Allocation
        [[nodiscard]] inline const Eigen::MatrixXf& allocation() const;
        [[nodiscard]] virtual inline const std::set<std::pair<unsigned int, unsigned int>>& mutexConstraints() const;
        [[nodiscard]] inline float scheduleBestMakespan() const;
        [[nodiscard]] inline float scheduleWorstMakespan() const;

        [[nodiscard]] inline float gamma() const;

        // Output from Task Planning
        [[nodiscard]] inline TraitsPlanView planTasks() const;
        [[nodiscard]] inline const std::shared_ptr<const Task>& planTask(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfPlanTasks() const;
        [[nodiscard]] virtual inline const std::set<std::pair<unsigned int, unsigned int>>& precedenceConstraints()
            const;

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& schedulerParameters() const;

        // Problem Inputs
        //// Tasks
        //// Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>>& robots() const;
        [[nodiscard]] inline const std::shared_ptr<const Robot>& robot(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfRobots() const;
        //// Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>>& multipleSpecies() const;
        [[nodiscard]] inline const std::shared_ptr<const Species>& individualSpecies(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfSpecies() const;
        //// Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& motionPlanners() const;
        [[nodiscard]] inline const std::shared_ptr<TraitsOmplMotionPlanner>& motionPlanner(unsigned int index) const;

        [[nodiscard]] inline SchedulerObjectiveType schedulerObjectiveType() const;

       protected:
        // From task allocation (order matters)
        std::set<std::pair<unsigned int, unsigned int>> m_mutex_constraints;  //!< Must be before m_allocation
        Eigen::MatrixXf m_allocation;                                         //!< Must be after m_mutex_constraints

        SchedulerObjectiveType m_schedule_objective_type;
        std::shared_ptr<const TraitsProblemInputs> m_traits_problem_inputs;

        friend class nlohmann::adl_serializer<std::shared_ptr<TraitsSchedulerProblemInputs>>;
    };

    // Inline functions
    float TraitsSchedulerProblemInputs::gamma() const
    {
        return m_traits_problem_inputs->gamma();
    }

    SchedulerObjectiveType TraitsSchedulerProblemInputs::schedulerObjectiveType() const
    {
       return m_schedule_objective_type;
    }

    const Eigen::MatrixXf& TraitsSchedulerProblemInputs::allocation() const
    {
        return m_allocation;
    }

    const std::set<std::pair<unsigned int, unsigned int>>& TraitsSchedulerProblemInputs::mutexConstraints() const
    {
        return m_mutex_constraints;
    }

    const std::shared_ptr<const TraitsProblemInputs>& TraitsSchedulerProblemInputs::getTraitsProblemInputs() const
    {
        return m_traits_problem_inputs;
    }

    TraitsPlanView TraitsSchedulerProblemInputs::planTasks() const
    {
        return m_traits_problem_inputs->planTasks();
    }

    const std::shared_ptr<const Task>& TraitsSchedulerProblemInputs::planTask(unsigned int index) const
    {
        return m_traits_problem_inputs->planTask(index);
    }

    unsigned int TraitsSchedulerProblemInputs::numberOfPlanTasks() const
    {
        return m_traits_problem_inputs->numberOfPlanTasks();
    }

    const std::set<std::pair<unsigned int, unsigned int>>& TraitsSchedulerProblemInputs::precedenceConstraints() const
    {
        return m_traits_problem_inputs->precedenceConstraints();
    }

    const std::shared_ptr<const ParametersBase>& TraitsSchedulerProblemInputs::schedulerParameters() const
    {
        return m_traits_problem_inputs->schedulerParameters();
    }

    const std::vector<std::shared_ptr<const Robot>>& TraitsSchedulerProblemInputs::robots() const
    {
        return m_traits_problem_inputs->robots();
    }

    const std::shared_ptr<const Robot>& TraitsSchedulerProblemInputs::robot(unsigned int index) const
    {
        return m_traits_problem_inputs->robot(index);
    }

    unsigned int TraitsSchedulerProblemInputs::numberOfRobots() const
    {
        return m_traits_problem_inputs->numberOfRobots();
    }

    const std::vector<std::shared_ptr<const Species>>& TraitsSchedulerProblemInputs::multipleSpecies() const
    {
        return m_traits_problem_inputs->multipleSpecies();
    }

    const std::shared_ptr<const Species>& TraitsSchedulerProblemInputs::individualSpecies(unsigned int index) const
    {
        return m_traits_problem_inputs->individualSpecies(index);
    }

    unsigned int TraitsSchedulerProblemInputs::numberOfSpecies() const
    {
        return m_traits_problem_inputs->numberOfSpecies();
    }

    const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& TraitsSchedulerProblemInputs::motionPlanners() const
    {
        return m_traits_problem_inputs->motionPlanners();
    }

    const std::shared_ptr<TraitsOmplMotionPlanner>& TraitsSchedulerProblemInputs::motionPlanner(unsigned int index) const
    {
        return m_traits_problem_inputs->motionPlanner(index);
    }

    float TraitsSchedulerProblemInputs::scheduleBestMakespan() const
    {
        return m_traits_problem_inputs->scheduleBestMakespan();
    }

    float TraitsSchedulerProblemInputs::scheduleWorstMakespan() const
    {
        return m_traits_problem_inputs->scheduleWorstMakespan();
    }
}  // namespace traits

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<traits::TraitsSchedulerProblemInputs>>
    {
        static std::shared_ptr<traits::TraitsSchedulerProblemInputs> from_json(const nlohmann::json& j);
    };

}  // namespace nlohmann