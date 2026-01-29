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
// External
#include <nlohmann/json.hpp>

namespace traits
{
    // Forward Declarations
    class ProblemInputs;

    /*!
     * \brief Base class for statistics recorded during search
     *
     * \note This class must be derived from
     */
    class SearchStatisticsBase
    {
    public:
        virtual ~SearchStatisticsBase() = default;

        //! \brief Serializes to json
        [[nodiscard]] virtual nlohmann::json serializeToJson(const std::shared_ptr<const ProblemInputs>& problem_inputs) const = 0;

        //! \brief Prints a human-readable representation of the statistics to a stream
        virtual std::ostream& print(std::ostream& os) const = 0;

        //! \brief Prints a human-readable representation of the statistics to the screen
        void printStatistics() const;

    protected:
        //! Constructor
        explicit SearchStatisticsBase(const std::string& time_name);

        std::string m_timer_name;
    };

    /*!
     * \brief Concept to force a type to derive from SearchStatisticsBase
     *
     * \tparam T A derivative of SearchStatisticsBase
     */
    template <typename T>
    concept SearchStatisticsDeriv = std::derived_from<T, SearchStatisticsBase>;
}  // namespace traits
