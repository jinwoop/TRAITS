/* Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Jinwoo Park
 *
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
#include "traits/task_allocation/nlp/trait_distributor_result.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

// Global
#include <cassert>

namespace traits
{
    TraitDistributorResult::TraitDistributorResult(
            const std::shared_ptr<const TaskTraitAllocation> &task_trait_allocations)
            : m_task_trait_allocations(task_trait_allocations)
            , m_failure_reason(nullptr)
    {
        assert(task_trait_allocations);
    }

    TraitDistributorResult::TraitDistributorResult(const std::shared_ptr<const FailureReason>& failure_reason)
            : m_task_trait_allocations(nullptr)
            , m_failure_reason(failure_reason)
    {
        assert(failure_reason);
    }
}  // namespace traits
