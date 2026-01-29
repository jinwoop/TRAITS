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
#include <fstream>
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "traits/common/search/greedy_best_first_search/greedy_best_first_search.hpp"
#include "traits/common/search/hash_memoization.hpp"
#include "traits/problem_inputs/traits_problem_inputs.hpp"
#include "traits/task_allocation/incremental_allocation_generator.hpp"
#include "traits/task_allocation/incremental_task_allocation_node.hpp"
#include "traits/task_allocation/statistics.hpp"
#include "traits/task_allocation/dynamic_time_extended_task_allocation_quality.hpp"
#include "traits/task_allocation/traits_improvement_pruning.hpp"
#include "traits/task_allocation/infeasible_pruning.hpp"
#include "traits/task_allocation/zero_apr_check.hpp"

namespace traits
{
    namespace detail
    {
        /*!
         * \brief Hack for designated initializers
         */
        struct TraitsParametersImpl
        {
            std::shared_ptr<const TraitsProblemInputs> problem_inputs;
            std::shared_ptr<const HeuristicBase<TraitsIncrementalTaskAllocationNode>> heuristic                    = nullptr;
            std::shared_ptr<const SuccessorGeneratorBase<TraitsIncrementalTaskAllocationNode>> successor_generator = nullptr;
            std::shared_ptr<const GoalCheckBase<TraitsIncrementalTaskAllocationNode>> goal_check                   = nullptr;
            std::shared_ptr<const MemoizationBase<TraitsIncrementalTaskAllocationNode>> memoization                = nullptr;
            std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>> pre_pruning_method             = nullptr;
            std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>> post_pruning_method            = nullptr;
        };
    }  // namespace detail

    /*!
     * \brief The Incremental Task Allocation Graph Search
     *
     * A heuristic search used for trait-based time extended task allocation problems
     *
     * \tparam The heuristic to be used during search
     *
     */
    class TRAITS : public GreedyBestFirstSearch<TraitsIncrementalTaskAllocationNode, Statistics>
    {
        using Base_ = GreedyBestFirstSearch<TraitsIncrementalTaskAllocationNode, Statistics>;

       public:
        /*!
         * \brief Constructor
         *
         * \param problem_inputs
         * \param heuristic
         * \param successor_generator
         * \param goal_check
         * \param memoization
         * \param pre_pruning_method
         * \param post_pruning_method
         */
        explicit TRAITS(
            const std::shared_ptr<const TraitsProblemInputs>& problem_inputs,
            const std::shared_ptr<const HeuristicBase<TraitsIncrementalTaskAllocationNode>>& heuristic,
            const std::shared_ptr<const SuccessorGeneratorBase<TraitsIncrementalTaskAllocationNode>>& successor_generator,
            const std::shared_ptr<const GoalCheckBase<TraitsIncrementalTaskAllocationNode>>& goal_check,
            const std::shared_ptr<const MemoizationBase<TraitsIncrementalTaskAllocationNode>>& memoization,
            const std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>>& pre_pruning_method,
            const std::shared_ptr<PruningMethodBase<TraitsIncrementalTaskAllocationNode>>& post_pruning_method);

        /*!
         * \brief Factory function
         *
         * This can be considered the factory function for the default ITAGS
         *
         * \param problem_inputs Inputs from the problem
         */
        explicit TRAITS(const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
                : TRAITS(problem_inputs,
                       std::make_shared<const DynamicTimeExtendedTaskAllocationQuality>(problem_inputs),
                       std::make_shared<const IncrementalAllocationGenerator>(problem_inputs),
                       std::make_shared<const TraitsZeroAprCheck>(problem_inputs),
                       std::make_shared<const HashMemoization<TraitsIncrementalTaskAllocationNode>>(),
                       std::make_shared<TraitsTraitsImprovementPruning>(problem_inputs),
                       std::make_shared<TraitsInfeasiblePruning>(problem_inputs))
        {}

        /*!
         * \brief Factory function
         *
         * \param parameters Designated parameters handler for customizing the search
         */
        explicit TRAITS(const detail::TraitsParametersImpl& parameters)
            : Base_{parameters.problem_inputs->TraitsParameters(),
                    {.heuristic =
                         parameters.heuristic != nullptr
                             ? parameters.heuristic
                             : std::make_shared<const DynamicTimeExtendedTaskAllocationQuality>(parameters.problem_inputs),
                     .successor_generator =
                         parameters.successor_generator != nullptr
                             ? parameters.successor_generator
                             : std::make_shared<const IncrementalAllocationGenerator>(parameters.problem_inputs),
                     .goal_check         = parameters.goal_check != nullptr
                                               ? parameters.goal_check
                                               : std::make_shared<const TraitsZeroAprCheck>(parameters.problem_inputs),
                     .memoization        = parameters.memoization != nullptr
                                               ? parameters.memoization
                                               : std::make_shared<const HashMemoization<TraitsIncrementalTaskAllocationNode>>(),
                     .prepruning_method  = parameters.pre_pruning_method != nullptr
                                               ? parameters.pre_pruning_method
                                               : std::make_shared<TraitsTraitsImprovementPruning>(parameters.problem_inputs),
                     .postpruning_method = parameters.post_pruning_method != nullptr
                                               ? parameters.post_pruning_method
                                               : std::make_shared<NullPruningMethod<TraitsIncrementalTaskAllocationNode>>()}}
            , m_problem_inputs(parameters.problem_inputs)
        {}

        /*!
         * \brief Factory function
         *
         * \param parameters Designated parameters handler for customizing the search
         */
        explicit TRAITS(detail::TraitsParametersImpl&& parameters)
            : Base_{parameters.problem_inputs->TraitsParameters(),
                    {.heuristic =
                         parameters.heuristic != nullptr
                             ? std::move(parameters.heuristic)
                             : std::make_shared<const DynamicTimeExtendedTaskAllocationQuality>(parameters.problem_inputs),
                     .successor_generator =
                         parameters.successor_generator != nullptr
                             ? std::move(parameters.successor_generator)
                             : std::make_shared<const IncrementalAllocationGenerator>(parameters.problem_inputs),
                     .goal_check         = parameters.goal_check != nullptr
                                               ? std::move(parameters.goal_check)
                                               : std::make_shared<const TraitsZeroAprCheck>(parameters.problem_inputs),
                     .memoization        = parameters.memoization != nullptr
                                               ? std::move(parameters.memoization)
                                               : std::make_shared<const HashMemoization<TraitsIncrementalTaskAllocationNode>>(),
                     .prepruning_method  = parameters.pre_pruning_method != nullptr
                                               ? std::move(parameters.pre_pruning_method)
                                               : std::make_shared<TraitsTraitsImprovementPruning>(parameters.problem_inputs),
                     .postpruning_method = parameters.post_pruning_method != nullptr
                                               ? std::move(parameters.post_pruning_method)
                                               : std::make_shared<NullPruningMethod<TraitsIncrementalTaskAllocationNode>>()}}
            , m_problem_inputs(parameters.problem_inputs)
        {}

        /*!
         * \returns Whether the specified problem can be allocated
         */
        [[nodiscard]] bool isAllocatable() const;

        /*!
         * \copydoc BestFirstSearchBase
         */
        [[nodiscard]] std::shared_ptr<TraitsIncrementalTaskAllocationNode> createRootNode() override;

       private:
        std::shared_ptr<const TraitsProblemInputs> m_problem_inputs;
    };
}  // namespace traits