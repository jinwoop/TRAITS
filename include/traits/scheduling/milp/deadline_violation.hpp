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
// Local
#include "traits/common/utilities/failure_reason.hpp"
// endregion

namespace traits
{
    //! Represents that a deadline violation was the reason for failure
    class DeadlineViolation : public FailureReason
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        constexpr DeadlineViolation() = default;
        //! Copy Constructor
        DeadlineViolation(const DeadlineViolation&) = default;
        //! Move Constructor
        DeadlineViolation(DeadlineViolation&&) noexcept = default;
        //! Destructor
        ~DeadlineViolation() = default;
        //! Copy Assignment Operator
        DeadlineViolation& operator=(const DeadlineViolation&) = default;
        //! Move Assignment Operator
        DeadlineViolation& operator=(DeadlineViolation&&) noexcept = default;
        // endregion
    };  // class DeadlineViolation
}  // namespace traits