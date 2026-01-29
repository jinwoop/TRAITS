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
#include <cstdint>
// External
#include <nlohmann/json.hpp>
// endregion

namespace traits
{
    //!
    enum class TimePointType : uint8_t
    {
        e_start,
        e_completion,
        e_null
    };

    //!
    class TimePoint
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        TimePoint() = default;
        //! Copy Constructor
        TimePoint(const TimePoint&) = default;
        //! Move Constructor
        TimePoint(TimePoint&&) noexcept = default;
        //! Destructor
        ~TimePoint() = default;
        //! Copy Assignment Operator
        TimePoint& operator=(const TimePoint&) = default;
        //! Move Assignment Operator
        TimePoint& operator=(TimePoint&&) noexcept = default;
        // endregion

        /*! Constructor
         *
         * \param timepoint_type
         * \param task
         */
        explicit TimePoint(TimePointType timepoint_type, unsigned int task);

        //! \returns
        [[nodiscard]] inline TimePointType timepointType() const;

        //! \returns
        [[nodiscard]] inline unsigned int task() const;

    private:
        TimePointType m_type;
        unsigned int m_task;
    };  // class Timepoint

    // region Inline Functions
    TimePointType TimePoint::timepointType() const
    {
        return m_type;
    }

    unsigned int TimePoint::task() const
    {
        return m_task;
    }
    // endregion

    // region json
    void from_json(const nlohmann::json& j, TimePoint& t);
    void to_json(nlohmann::json& j, const TimePoint& t);
    // endregion
}  // namespace grstapse