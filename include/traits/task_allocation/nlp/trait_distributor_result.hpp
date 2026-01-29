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
#pragma once

// Global
#include <memory>

namespace traits
{
    // Forward Declarations
    class TaskTraitAllocation;
    class FailureReason;

    /*!
     * The result of a scheduling algorithm
     */
    class TraitDistributorResult
    {
    public:
        //! Constructor
        explicit TraitDistributorResult(const std::shared_ptr<const TaskTraitAllocation>& trait_allocation);

        //! Constructor
        explicit TraitDistributorResult(const std::shared_ptr<const FailureReason>& failure_reason);

        //! \returns Whether the nonlinear program successfully found a trait allocation
        [[nodiscard]] inline bool success() const;

        //! \returns The found trait allocation
        [[nodiscard]] inline const std::shared_ptr<const TaskTraitAllocation>& taskTraitAllocation() const;

        //! \returns Whether the nonlinear program failed found a trait allocation
        [[nodiscard]] inline bool failed() const;

        //! \returns The reason for failure
        [[nodiscard]] inline const std::shared_ptr<const FailureReason>& failureReason() const;

    private:
        std::shared_ptr<const TaskTraitAllocation> m_task_trait_allocations;
        std::shared_ptr<const FailureReason> m_failure_reason;
    };

    // Inline Functions
    bool TraitDistributorResult::success() const
    {
        return m_task_trait_allocations!= nullptr;
    }

    const std::shared_ptr<const TaskTraitAllocation>& TraitDistributorResult::taskTraitAllocation() const {
        return m_task_trait_allocations;
    }

    bool TraitDistributorResult::failed() const
    {
        return m_failure_reason != nullptr;
    }

    const std::shared_ptr<const FailureReason>& TraitDistributorResult::failureReason() const
    {
        return m_failure_reason;
    }
}  // namespace traits
