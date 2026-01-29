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
#include <nlohmann/json.hpp>
// Local
#include "traits/scheduling/deadline_base.hpp"
#include "traits/scheduling/timepoint.hpp"
// endregion

namespace traits
{
    //! Container for the upperbound on the temporal difference between two timepoints
    class RelativeDeadline : public DeadlineBase
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        RelativeDeadline() = delete;
        //! Copy Constructor
        RelativeDeadline(const RelativeDeadline&) = default;
        //! Move Constructor
        RelativeDeadline(RelativeDeadline&&) noexcept = default;
        //! Destructor
        ~RelativeDeadline() = default;
        //! Copy Assignment Operator
        RelativeDeadline& operator=(const RelativeDeadline&) = default;
        //! Move Assignment Operator
        RelativeDeadline& operator=(RelativeDeadline&&) noexcept = default;
        // endregion

        /*! Constructor
         *
         * \param predecessor
         * \param successor
         * \param bound
         */
        explicit RelativeDeadline(const TimePoint& predecessor, const TimePoint& successor, float bound);

        [[nodiscard]] inline const TimePoint& predecessor() const;

        [[nodiscard]] inline const TimePoint& successor() const;

        [[nodiscard]] inline float bound() const;

    private:
        TimePoint m_predecessor;
        TimePoint m_successor;
        float m_bound;
    };

    // region Inline Functions
    const TimePoint& RelativeDeadline::predecessor() const
    {
        return m_predecessor;
    }
    const TimePoint& RelativeDeadline::successor() const
    {
        return m_successor;
    }
    float RelativeDeadline::bound() const
    {
        return m_bound;
    }
    // endregion

}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const traits::RelativeDeadline>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const traits::RelativeDeadline> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<const traits::RelativeDeadline>& d);
    };

    template <>
    struct adl_serializer<std::shared_ptr<traits::RelativeDeadline>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<traits::RelativeDeadline> from_json(const json& j);
        static void to_json(json& j, const std::shared_ptr<traits::RelativeDeadline>& d);
    };
}  // namespace nlohmann