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
#include <memory>
// Local
#include "traits/common/search/search_node_base.hpp"
#include "traits/common/utilities/noncopyable.hpp"

namespace traits
{
    /*!
     * \brief An interface for defining when to prune nodes from a search
     *
     * \tparam SearchNode A derivative of SearchNodeBase
     */
    template <SearchNodeDeriv SearchNode>
    class PruningMethodBase : private Noncopyable
    {
    public:
        //! \returns Whether to prune this node from the search
        [[nodiscard]] virtual bool operator()(const std::shared_ptr<const SearchNode>& node) const = 0;

    protected:
        PruningMethodBase() = default;
    };

    /*!
     * \brief
     *
     * \tparam T
     * \tparam SearchNode
     */
    template <typename T, typename SearchNode>
    concept PruningMethodDeriv = std::derived_from<T, PruningMethodBase<SearchNode>>;
}  // namespace traits
