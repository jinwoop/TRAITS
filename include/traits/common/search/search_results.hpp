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
#include <concepts>
#include <fstream>
#include <iomanip>
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "traits/common/search/search_node_base.hpp"
#include "traits/common/search/search_statistics_common.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/task_allocation/zero_apr_check.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

namespace traits
{
    class TraitsProblemInputs;
    /*!
     * \brief Container for the results of a search
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     * \tparam SearchStatistics A derivative of SearchStatisticsBase
     */
    template <SearchNodeDeriv SearchNode, SearchStatisticsDeriv SearchStatistics = SearchStatisticsCommon>
    class SearchResults
    {
    public:
        /*!
         * \brief Constructor
         *
         * \param goal The final node in the search
         * \param statistics The statistics from the search
         */
        explicit SearchResults(const std::shared_ptr<SearchNode>& goal,
                               const std::shared_ptr<SearchStatistics>& statistics)
                : m_goal(goal)
                , m_statistics(statistics)
        {}

        //! \returns Whether the goal was found during search
        [[nodiscard]] bool foundGoal() const
        {
            const auto& _m_goal = std::dynamic_pointer_cast<const TraitsIncrementalTaskAllocationNode>(m_goal);
            // The search returns a null goal when it exhausts the open set or hits its timeout
            // without ever reaching a goal node.
            if (_m_goal == nullptr) {
                return false;
            }
            if (_m_goal->taskTraitAllocation() == nullptr || _m_goal->schedule() == nullptr) {
                return false;
            }

            // NLP Trait Distributor and MILP Scheduler worked.
            const float traits_mismatch_error = _m_goal->taskTraitAllocation()->trait_deficiencies();
            const float trait_rates_mismatch_error = _m_goal->taskTraitAllocation()->trait_rate_deficiencies();

            return traits_mismatch_error + trait_rates_mismatch_error < 1e-2f;
        }

        //! \returns The goal found during a search
        [[nodiscard]] const std::shared_ptr<SearchNode>& goal()
        {
            return m_goal;
        }

        //! \returns The goal found during a search
        [[nodiscard]] std::shared_ptr<const SearchNode> goal() const
        {
            return m_goal;
        }

        //! \returns The statistics of the search that produced this result
        [[nodiscard]] const std::shared_ptr<SearchStatistics>& statistics()
        {
            return m_statistics;
        }

        //! \returns The statistics of the search that produced this result
        [[nodiscard]] std::shared_ptr<const SearchStatistics> statistics() const
        {
            return m_statistics;
        }

        //! Writes the results of the search to file
        void writeToFile(const std::string& filepath, const std::shared_ptr<const TraitsProblemInputs>& problem_inputs) const
        {
            nlohmann::json j;

            if(m_goal == nullptr)
            {
                // Nothing was found; record that rather than dereferencing a null goal.
                j[constants::k_full_solution] = false;
                j[constants::k_solution]      = nullptr;
                j[constants::k_statistics]    = m_statistics->serializeToJson(problem_inputs);
                std::ofstream out(filepath);
                out << std::setw(4) << j << std::endl;
                return;
            }

            TraitsZeroAprCheck goal_checker(problem_inputs);
            if(goal_checker.operator()(m_goal))
            {
                j[constants::k_full_solution] = true;
            } else {
                j[constants::k_full_solution] = false;
            }

            j[constants::k_solution]   = (m_goal != nullptr) ? m_goal->serializeToJson(problem_inputs) : nullptr;
            if(goal_checker.operator()(m_goal)) {
                j["task trait allocations"] =
                        m_goal->taskTraitAllocation()->serializeToJson(
                                std::dynamic_pointer_cast<const TraitsProblemInputs>(problem_inputs),
                                m_goal->allocation());
            } else{
                j["task_apr"] =
                        std::dynamic_pointer_cast<TraitsIncrementalTaskAllocationNode>(m_goal)->apr();
                j["task_apr_E"] =
                        std::dynamic_pointer_cast<TraitsIncrementalTaskAllocationNode>(m_goal)->taskTraitAllocation()->trait_deficiencies();
                j["task_apr_Edot"] =
                        std::dynamic_pointer_cast<TraitsIncrementalTaskAllocationNode>(m_goal)->taskTraitAllocation()->trait_rate_deficiencies();
            }
            j[constants::k_statistics] = m_statistics->serializeToJson(problem_inputs);

            std::ofstream out(filepath);
            out << std::setw(4) << j << std::endl;
        }

    private:
        std::shared_ptr<SearchNode> m_goal;
        std::shared_ptr<SearchStatistics> m_statistics;
    };
}  // namespace traits
