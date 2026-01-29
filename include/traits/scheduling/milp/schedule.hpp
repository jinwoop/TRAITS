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
#include <memory>
#include <string>
#include <tuple>
#include <vector>
// External
#include <nlohmann/json.hpp>
// Local
#include "traits/common/utilities/noncopyable.hpp"
// endregion

namespace traits
{
    // region Forward Declarations
    class TraitsSchedulerProblemInputs;
    // endregion
    /*!
     *
     * \class TraitsSchedule
     * \brief Container for a deterministic schedule for a set of tasks with constraints
     */
    class TraitsSchedule: public Noncopyable
    {
       public:
        // region Special Member Functions
        //! \brief Default Constructor
        TraitsSchedule()                                 = default;
        TraitsSchedule(const TraitsSchedule&)     = delete;
        TraitsSchedule(TraitsSchedule&&) noexcept = default;
        ~TraitsSchedule()                                = default;
        TraitsSchedule& operator=(const TraitsSchedule&) = delete;
        TraitsSchedule& operator=(TraitsSchedule&) noexcept = default;
        // endregion

        /*!
         * \brief Full Constructor
         *
         * \param makespan The total execution time for the schedule
         * \param time_points A list of the start/end timepoints for a set of tasks
         * \param precedence_set_mutex_constraints A list of the precedence set mutex constraints
         */
        TraitsSchedule(
            const float objective_ratio,
            const float makespan,
            const std::vector<std::pair<float, float>>& time_points,
            const std::vector<std::pair<unsigned int, unsigned int>>& precedence_set_mutex_constraints);

        //! \returns The makespan (or the total execution time) of the schedule
        [[nodiscard]] inline float makespan() const;

        //! \returns The total reward obtained by executing the schedule
        // [[nodiscard]] inline float reward() const;

        [[nodiscard]] inline float objectiveRatio() const;

        //! \returns A list of the precedence set mutex constraints
        [[nodiscard]] inline const std::vector<std::pair<unsigned int, unsigned int>>& precedenceSetMutexConstraints()
        const;

        /*!
         * \returns The list of time points where timepoints()[i].first is the start of the ith task/action and
         *          timepoints()[i].second is the end of the ith task/action
         */
        [[nodiscard]] inline const std::vector<std::pair<float, float>>& timepoints() const;

        //! \returns When the ith task starts
        [[nodiscard]] inline float taskStart(const unsigned int i) const;

        //! \returns When the ith task ends
        [[nodiscard]] inline float taskEnd(const unsigned int i) const;

        nlohmann::json serializeToJson(
            const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const;

       protected:
        float m_makespan;
        float m_makespan_ratio;

        std::vector<std::pair<float, float>> m_time_points;
        std::vector<std::pair<unsigned int, unsigned int>> m_precedence_set_mutex_constraints;
    };

    // Inline Functions
    float TraitsSchedule::objectiveRatio() const
    {
        return m_makespan_ratio;
    }

    float TraitsSchedule::makespan() const
    {
        return m_makespan;
    }

    const std::vector<std::pair<unsigned int, unsigned int>>& TraitsSchedule::precedenceSetMutexConstraints() const
    {
        return m_precedence_set_mutex_constraints;
    }
    const std::vector<std::pair<float, float>>& TraitsSchedule::timepoints() const
    {
        return m_time_points;
    }

    float TraitsSchedule::taskStart(const unsigned int i) const
    {
        return m_time_points[i].first;
    }

    float TraitsSchedule::taskEnd(const unsigned int i) const
    {
        return m_time_points[i].second;
    }

}  // namespace traits