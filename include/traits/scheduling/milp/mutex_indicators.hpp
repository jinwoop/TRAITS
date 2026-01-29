/* Graphically Recursive Simultaneous Task Allocation, Planning,
 * Scheduling, and Execution
 *
 * Copyright (C) 2020–2023
 *
 * Author: Andrew Messing
 * Author: Glen Neville
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
#include <set>
#include <unordered_map>
// External
#include <gurobi_c++.h>
// Local
#include "traits/common/utilities/hash_extension.hpp"

namespace traits
{
    // Forward Declarations
    class MsNameSchemeBase;
    class SchedulerProblemInputs;
    class TraitsSchedulerProblemInputs;

    /*!
     * \brief Handles operations that affect the mutex indicators for scheduling
     */
    class MutexIndicators
    {
       public:
        /*!
         * \brief Constructor
         *
         * \param mutex_constraints A set of mutex constraints on the tasks
         * \param precedence_constraints A set of precedence constraints on the tasks
         * \param isSearch true if this object is for search and not for bound calculations
         */
        MutexIndicators(const std::set<std::pair<unsigned int, unsigned int>>& mutex_constraints,
                        const std::set<std::pair<unsigned int, unsigned int>>& precedence_constraints,
                        const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                        bool isSearch = true);

        //! Constructor
        MutexIndicators(const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs,
                        const std::shared_ptr<const MsNameSchemeBase>& name_scheme,
                        bool isSearch = true);

        //! Creates the mutex indicator variables
        void createVariables(GRBModel& model);

        [[nodiscard]] inline bool contains(const std::pair<unsigned int, unsigned int>& p) const;

        //! \returns The indicator for the specified constraint
        [[nodiscard]] GRBVar& get(const std::pair<unsigned int, unsigned int>& p);

        //! \returns The map of forward mutex indicators
        [[nodiscard]] inline std::unordered_map<std::pair<unsigned int, unsigned int>, GRBVar>& indicators();

        //! \returns A list of the precedence reductions for the mutex constraints
        [[nodiscard]] std::vector<std::pair<unsigned int, unsigned int>> precedenceSet() const;

       private:
        const std::set<std::pair<unsigned int, unsigned int>>& m_precedence_constraints;
        std::shared_ptr<const MsNameSchemeBase> m_name_scheme;
        std::unordered_map<std::pair<unsigned int, unsigned int>, GRBVar> m_indicators;
        bool m_search;
    };

    // Inline Functions
    std::unordered_map<std::pair<unsigned int, unsigned int>, GRBVar>& MutexIndicators::indicators()
    {
        return m_indicators;
    }

    bool MutexIndicators::contains(const std::pair<unsigned int, unsigned int>& p) const
    {
        return m_indicators.contains(p);
    }

}  // namespace traits