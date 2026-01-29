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
#include <memory>
// External
#include <nlohmann/json.hpp>

// endregion

namespace traits
{
    /*!
     *
     */
    enum class DeadlineType : uint8_t
    {
        e_absolute,
        e_relative
    };

    //! Base class for deadlines (upperbounds on temporal difference between two timepoints)
    class DeadlineBase
    {
    public:
        // region Special Member Functions
        //! Default Constructor
        DeadlineBase() = delete;
        //! Copy Constructor
        DeadlineBase(const DeadlineBase&) = default;
        //! Move Constructor
        DeadlineBase(DeadlineBase&&) noexcept = default;
        //! Destructor
        virtual ~DeadlineBase() = default;
        //! Copy Assignment Operator
        DeadlineBase& operator=(const DeadlineBase&) = default;
        //! Move Assignment Operator
        DeadlineBase& operator=(DeadlineBase&&) noexcept = default;
        // endregion

        [[nodiscard]] inline DeadlineType deadlineType() const;

    protected:
        explicit DeadlineBase(DeadlineType t);

        DeadlineType m_type;
    };

    // class DeadlineBase

    // Inline Functions
    DeadlineType DeadlineBase::deadlineType() const
    {
        return m_type;
    }
}  // namespace grstapse

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<const traits::DeadlineBase>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<const traits::DeadlineBase> from_json(const json& j);
        //! to_json
        static void to_json(json& j, const std::shared_ptr<const traits::DeadlineBase>& d);
    };

    template <>
    struct adl_serializer<std::shared_ptr<traits::DeadlineBase>>
    {
        //! Non-default constructable from_json
        static std::shared_ptr<traits::DeadlineBase> from_json(const json& j);
        //! to_json
        static void to_json(json& j, const std::shared_ptr<traits::DeadlineBase>& d);
    };
}  // namespace nlohmann