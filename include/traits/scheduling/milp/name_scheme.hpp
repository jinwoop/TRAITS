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

// Local
#include "traits/scheduling/milp/ms_name_scheme_base.hpp"

#include <stdexcept>

namespace traits
{
    /*!
     * Main name scheme for the standard DMScheduler
     */
    class NameScheme : public MsNameSchemeBase
    {
       public:
            //! \returns The name for the makespan variable
            [[nodiscard]] std::string createMakespanVariableName() const final override;

            //! \returns The name for the task start MILP variable for \p task_nr
            [[nodiscard]] std::string createTaskStartName(unsigned int task_nr) const;

            //! \returns The name for the task MILP variable for \p task_nr
            [[nodiscard]] std::string createTaskEndName(unsigned int task_nr) const;

            //! \returns The name of the transition duration variable for \p i and \p j
            [[nodiscard]] std::string createTransitionDurationVariableName(unsigned int i,
                                                                           unsigned int j) const;

            //! \returns The name for the task duration constraint for \p task_nr
            [[nodiscard]] std::string createTaskDurationConstraintName(unsigned int task_nr) const;

            //! \returns The name of the task lowerbound constraint for \p task_nr
            [[nodiscard]] std::string createTaskStartLowerBoundConstraintName(unsigned int task_nr) const;

            //! \returns The name of the task upperbound constraint for \p task_nr
            [[nodiscard]] std::string createTaskEndUpperBoundConstraintName(unsigned int task_nr) const;

            //! \returns The name for the precedence constraint from \p i to \p j
            [[nodiscard]] std::string createPrecedenceConstraintName(unsigned int i, unsigned int j) const;

            //! \returns The name for the mutex constraint between \p i and \p j when it is resolve as \p i -> \p j
            [[nodiscard]] std::string createMutexConstraintName(unsigned int first, unsigned int second) const;

            [[nodiscard]] std::string createObjectiveMakespanRatioName() const;
            [[nodiscard]] std::string createObjectiveMakespanRatioConstraintName() const;
            [[nodiscard]] std::string createTaskEndTimeConstraintName(unsigned int task_nr) const;

            [[nodiscard]] std::string createAbsoluteDeadlineConstraintStartName(unsigned int task_nr) const;
            [[nodiscard]] std::string createAbsoluteDeadlineConstraintCompletionName(unsigned int task_nr) const;
            [[nodiscard]] std::string createRelativeDeadlineConstraintName(unsigned int predecessor,
                                                                           unsigned int successor) const;


        //! \returns The name for the lowerbound constraint on the transition duration
            [[nodiscard]] std::string createTransitionDurationLowerBoundConstraintName(
                    unsigned int first,
                    unsigned int second) const;

            //! \returns The name for the makespan constraint
            [[nodiscard]] std::string createMakespanConstraintName(unsigned int task_nr) const;

    };

}  // namespace traits