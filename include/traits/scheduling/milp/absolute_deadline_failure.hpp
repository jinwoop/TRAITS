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
// External
// Local
// endregion

namespace traits
{
    // region Forward Declarations
    // endregion

    /*!
     * \class AbsoluteDeadlineFailure
     * \brief
     */
    class AbsoluteDeadlineFailure
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        AbsoluteDeadlineFailure() = default;
        //! Copy Constructor
        AbsoluteDeadlineFailure(const AbsoluteDeadlineFailure&) = default;
        //! Move Constructor
        AbsoluteDeadlineFailure(AbsoluteDeadlineFailure&&) noexcept = default;
        //! Destructor
        ~AbsoluteDeadlineFailure() = default;
        //! Copy Assignment Operator
        AbsoluteDeadlineFailure& operator=(const AbsoluteDeadlineFailure&) = default;
        //! Move Assignment Operator
        AbsoluteDeadlineFailure& operator=(AbsoluteDeadlineFailure&&) noexcept = default;
        // endregion
    };  // class AbsoluteDeadlineFailure
}  // namespace traits