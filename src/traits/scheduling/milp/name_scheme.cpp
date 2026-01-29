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
#include "traits/scheduling/milp/name_scheme.hpp"

// External
#include <fmt/format.h>
// Local
#include "traits/common/utilities/constants.hpp"

namespace traits
{
    std::string NameScheme::createTaskStartName(unsigned int task_nr) const
    {
        return fmt::format("ts_{0:d}", task_nr);
    }

    std::string NameScheme::createTaskEndName(unsigned int task_nr) const
    {
        return fmt::format("te_{0:d}", task_nr);
    }

    std::string NameScheme::createTransitionDurationVariableName(unsigned int first,
                                                                     unsigned int second) const
    {
        return fmt::format("trd_({0:d},{1:d})", first, second);
    }

    std::string NameScheme::createMutexConstraintName(unsigned int first,
                                                          unsigned int second) const
    {
        return fmt::format("mi_({0:d},{1:d})", first, second);
    }

    std::string NameScheme::createMakespanVariableName() const
    {
        return constants::k_makespan;
    }

    std::string NameScheme::createTaskDurationConstraintName(unsigned int task_nr) const
    {
        return fmt::format("tdc_{0:d}", task_nr);
    }

    std::string NameScheme::createTaskStartLowerBoundConstraintName( unsigned int task_nr) const
    {
        return fmt::format("tlbc_{0:d}", task_nr);
    }

    std::string NameScheme::createTaskEndUpperBoundConstraintName( unsigned int task_nr) const
    {
        return fmt::format("tubc_{0:d}", task_nr);
    }

    std::string NameScheme::createPrecedenceConstraintName(unsigned int first,
                                                               unsigned int second) const
    {
        return fmt::format("pc_({0:d},{1:d})", first, second);
    }

    std::string NameScheme::createTransitionDurationLowerBoundConstraintName(unsigned int first,
                                                                             unsigned int second) const
    {
        return fmt::format("tdlbc_({0:d},{1:d})", first, second);
    }

    std::string NameScheme::createMakespanConstraintName(unsigned int task_nr) const
    {
        return fmt::format("mkc_{0:d}", task_nr);
    }

    std::string NameScheme::createObjectiveMakespanRatioName() const
    {
        return fmt::format("makespan_ratio_objective");
    }

    std::string NameScheme::createTaskEndTimeConstraintName(unsigned int task_nr) const
    {
        return fmt::format("tetc_{0:d}", task_nr);
    }

    std::string NameScheme::createObjectiveMakespanRatioConstraintName() const
    {
        return fmt::format("obj_Mrc");
    }

    std::string NameScheme::createAbsoluteDeadlineConstraintStartName(unsigned int task_nr) const
    {
       return fmt::format("abs_dl_{}_s", task_nr);
    }

    // Below is equivalent to upper bound constraint
    std::string NameScheme::createAbsoluteDeadlineConstraintCompletionName(unsigned int task_nr) const
    {
        return fmt::format("abs_dl_{}_c", task_nr);
    }

    std::string NameScheme::createRelativeDeadlineConstraintName(unsigned int predecessor,
                                                                     unsigned int successor) const
    {
        return fmt::format("rel_dl_{}_{}", predecessor, successor);
    }
}  // namespace traits