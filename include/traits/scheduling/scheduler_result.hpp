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

namespace traits
{
    // Forward Declarations
    class TraitsSchedule;
    class FailureReason;

    /*!
     * The result of a scheduling algorithm
     */
    class TraitsSchedulerResult
    {
       public:
        //! Constructor
        explicit TraitsSchedulerResult(const std::shared_ptr<const TraitsSchedule>& schedule);

        //! Constructor
        explicit TraitsSchedulerResult(const std::shared_ptr<const FailureReason>& failure_reason);

        //! \returns Whether the scheduler successfully found a schedule
        [[nodiscard]] inline bool success() const;

        //! \returns The found schedule
        [[nodiscard]] inline const std::shared_ptr<const TraitsSchedule>& schedule() const;

        //! \returns Whether the scheduler failed found a schedule
        [[nodiscard]] inline bool failed() const;

        //! \returns The reason for failure
        [[nodiscard]] inline const std::shared_ptr<const FailureReason>& failureReason() const;

       private:
        std::shared_ptr<const TraitsSchedule> m_schedule;
        std::shared_ptr<const FailureReason> m_failure_reason;
    };

    // Inline Functions
    bool TraitsSchedulerResult::success() const
    {
        return m_schedule != nullptr;
    }

    const std::shared_ptr<const TraitsSchedule>& TraitsSchedulerResult::schedule() const
    {
        return m_schedule;
    }

    bool TraitsSchedulerResult::failed() const
    {
        return m_failure_reason != nullptr;
    }

    const std::shared_ptr<const FailureReason>& TraitsSchedulerResult::failureReason() const
    {
        return m_failure_reason;
    }
}  // namespace traits