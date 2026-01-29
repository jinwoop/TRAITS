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
#include "traits/scheduling/deadline_base.hpp"
#include "traits/scheduling/timepoint.hpp"
// endregion

namespace traits
{
    //! Container for the upperbound on a timepoint (from time=0)
    class AbsoluteDeadline : public DeadlineBase
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        AbsoluteDeadline() = delete;
        //! Copy Constructor
        AbsoluteDeadline(const AbsoluteDeadline&) = default;
        //! Move Constructor
        AbsoluteDeadline(AbsoluteDeadline&&) noexcept = default;
        //! Destructor
        ~AbsoluteDeadline() = default;
        //! Copy Assignment Operator
        AbsoluteDeadline& operator=(const AbsoluteDeadline&) = default;
        //! Move Assignment Operator
        AbsoluteDeadline& operator=(AbsoluteDeadline&&) noexcept = default;
        // endregion

        /*! Constructor
         *
         * \param timepoint
         * \param bound
         */
        explicit AbsoluteDeadline(const TimePoint& timepoint, float bound);

        //! \returns
        [[nodiscard]] inline const TimePoint& timepoint() const;

        //! \returns
        [[nodiscard]] inline float bound() const;

    private:
        TimePoint m_timepoint;
        float m_bound;
    };  // class AbsoluteDeadline

    // region Inline Functions
    const TimePoint& AbsoluteDeadline::timepoint() const
    {
        return m_timepoint;
    }

    float AbsoluteDeadline::bound() const
    {
        return m_bound;
    }
    // endregion
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const traits::AbsoluteDeadline>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const traits::AbsoluteDeadline> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const traits::AbsoluteDeadline>& d);
    };

    template <>
    struct adl_serializer<std::shared_ptr<traits::AbsoluteDeadline>>
{
    //! Non-default constructable from_json
    static std::shared_ptr<traits::AbsoluteDeadline> from_json(const json& j);
    static void to_json(json& j, const std::shared_ptr<traits::AbsoluteDeadline>& d);
};
}  // namespace nlohmann