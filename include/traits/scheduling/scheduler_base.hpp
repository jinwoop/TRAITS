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
// External
#include <nlohmann/json.hpp>
// Local
#include "traits/common/utilities/noncopyable.hpp"
#include "traits/common/utilities/timer.hpp"
// endregion

namespace traits
{
    // region Forward Declarations
    class TraitsSchedulerProblemInputs;
    class SchedulerParameters;
    class TraitsSchedulerResult;
    // endregion

    //! \brief Abstract base class for a scheduling algorithm
    class TraitsSchedulerBase : public Noncopyable
    {
       public:
        // region Special Member Functions
        TraitsSchedulerBase()                         = delete;
        TraitsSchedulerBase(const TraitsSchedulerBase&)     = delete;
        TraitsSchedulerBase(TraitsSchedulerBase&&) noexcept = default;
        virtual ~TraitsSchedulerBase()                = default;
        TraitsSchedulerBase& operator=(const TraitsSchedulerBase&) = delete;
        TraitsSchedulerBase& operator=(TraitsSchedulerBase&&) noexcept = default;
        // endregion

        /*!
         * \brief Solves the scheduling problem
         *
         * \returns The result of attempting to solveMilp the problem
         */
        [[nodiscard]] std::shared_ptr<const TraitsSchedulerResult> solve();

        //! \returns The number of time that scheduling has failed
        [[nodiscard]] static unsigned int numFailures();

       protected:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs The inputs for a scheduling problem
         * \param parameters The parameters for configuring the scheduling algorithm
         */
        explicit TraitsSchedulerBase(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs);

        /*!
         * \brief Solves the scheduling problem
         *
         * \returns The result of attempting to solveMilp the problem
         */
        [[nodiscard]] virtual std::shared_ptr<const TraitsSchedulerResult> computeSchedule() = 0;

        std::shared_ptr<const TraitsSchedulerProblemInputs> m_problem_inputs;

        static unsigned int s_num_failures;
    };

    /*!
     * Concept to force a template parameter to derive from TraitsSchedulerBase
     *
     * \tparam T A derivative of TraitsSchedulerBase
     */
    template <typename T>
    concept TraitsSchedulerDeriv = std::derived_from<T, TraitsSchedulerBase>;
}  // namespace traits