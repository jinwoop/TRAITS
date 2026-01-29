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
#include <memory>
// External
#include <nlohmann/json.hpp>
// Local
#include "traits/common/utilities/noncopyable.hpp"
// endregion

namespace traits
{
    // region Forward Declaration
    class GrstapsProblemInputs;
    class ItagsProblemInputs;
    class SchedulerProblemInputs;
    class TraitsProblemInputs;
    class TraitsSchedulerProblemInputs;
    // endregion

    //! Base class for problem inputs
    struct ProblemInputs : private Noncopyable
    {
        // region Special Member Functions
       protected:
        constexpr ProblemInputs() = default;

       public:
        ProblemInputs(const ProblemInputs&)     = delete;
        ProblemInputs(ProblemInputs&&) noexcept = default;
        virtual ~ProblemInputs()                = default;
        ProblemInputs& operator=(const ProblemInputs&) = delete;
        ProblemInputs& operator=(ProblemInputs&&) noexcept = default;
        // endregion

       protected:
        //! Tag to make a protected constructor that can still be used with std::make_shared
        constexpr static const struct ThisIsProtectedTag { }
        s_this_is_protected_tag = ThisIsProtectedTag{};

        friend struct nlohmann::adl_serializer<std::shared_ptr<GrstapsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<ItagsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<SchedulerProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<TraitsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<TraitsSchedulerProblemInputs>>;
    };

}  // namespace traits