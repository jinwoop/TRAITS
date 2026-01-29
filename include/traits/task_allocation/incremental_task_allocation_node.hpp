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
#include <optional>
// External
#include <eigen3/Eigen/Core>
// Local
#include "traits/common/search/greedy_best_first_search/greedy_best_first_search_node_base.hpp"
#include "traits/common/utilities/matrix_dimensions.hpp"
#include "traits/task_allocation/assignment.hpp"
#include "traits/task_allocation/nlp/trait_distributor.hpp"

namespace traits
{
    // Forward Declaration
//    class ScheduleBase;
    class TraitsSchedule;
    class TaskTraitAllocation;

    //! \brief A node that contains an allocation of agents to tasks
    class TraitsIncrementalTaskAllocationNode : public GreedyBestFirstSearchNodeBase<TraitsIncrementalTaskAllocationNode>
    {
        using Base_ = GreedyBestFirstSearchNodeBase<TraitsIncrementalTaskAllocationNode>;

       public:
        /*!
         * \brief Constructor for the root node
         * \param dimensions The dimensions of the allocation matrix this node represents
         */
        explicit TraitsIncrementalTaskAllocationNode(const MatrixDimensions& dimensions);
        explicit TraitsIncrementalTaskAllocationNode(const MatrixDimensions& dimensions,
                                                   float apr);

        /*!
         * \brief Constructor for any node except the root node
         *
         * \param assignment The last assignment for the allocation matrix this node represents
         * \param parent The parent of this node
         */
        TraitsIncrementalTaskAllocationNode(const Assignment& assignment,
                                          const std::shared_ptr<const TraitsIncrementalTaskAllocationNode>& parent);

        //! \returns The last assigment (robot, task)
        [[nodiscard]] inline const std::optional<Assignment>& lastAssigment() const;

        //! \returns The dimensions of the allocation matrix (MxN)
        [[nodiscard]] const MatrixDimensions& matrixDimensions() const;

        /*!
         * \returns The allocation contained by this node
         *
         * \note Virtual for unit tests
         */
        virtual Eigen::MatrixXf allocation() const;

        //! Sets the schedule for this node
        inline void setSchedule(const std::shared_ptr<const TraitsSchedule>& schedule);

        //! \returns The schedule computed for this node if RSQ was run
        [[nodiscard]] inline const std::shared_ptr<const TraitsSchedule>& schedule() const;

        //! Sets the trait distribution object for this node
        inline void setTaskTraitAllocation(
                const std::shared_ptr<const TaskTraitAllocation>& task_trait_allocations) const;

        //! \returns The trait distributions for this allocation if feasible solution exists
        [[nodiscard]] inline const std::shared_ptr<const TaskTraitAllocation> taskTraitAllocation() const;

        //! \returns A hash for this node
        [[nodiscard]] unsigned int hash() const final override;

        [[nodiscard]] inline float apr() const;
        inline void set_apr(float apr) const;
        [[nodiscard]] inline float nsq() const;
        inline void set_nsq(float nsq) const;


        //! \copydoc SearchNodeBase
        [[nodiscard]] nlohmann::json serializeToJson(
            const std::shared_ptr<const ProblemInputs>& problem_inputs) const override;

       private:
        std::optional<Assignment> m_last_assigment;
        std::optional<MatrixDimensions> m_matrix_dimensions;
        std::shared_ptr<const TraitsSchedule> m_schedule;
        mutable std::shared_ptr<const TaskTraitAllocation> m_task_trait_allocations;  // a result of the Nonlinear Program

        mutable float m_apr;
        mutable float m_nsq;

        static unsigned int s_next_id;
    };

    // Inline functions
    float TraitsIncrementalTaskAllocationNode::apr() const
    {
        return m_apr;
    }

    void TraitsIncrementalTaskAllocationNode::set_apr(float apr) const
    {
        m_apr = apr;
    }

    float TraitsIncrementalTaskAllocationNode::nsq() const
    {
        return m_nsq;
    }

    void TraitsIncrementalTaskAllocationNode::set_nsq(float nsq) const
    {
        m_nsq = nsq;
    }

    const std::optional<Assignment>& TraitsIncrementalTaskAllocationNode::lastAssigment() const
    {
        return m_last_assigment;
    }

    void TraitsIncrementalTaskAllocationNode::setSchedule(const std::shared_ptr<const TraitsSchedule>& schedule)
    {
        m_schedule = schedule;
    }

    const std::shared_ptr<const TraitsSchedule>& TraitsIncrementalTaskAllocationNode::schedule() const
    {
        return m_schedule;
    }

    void TraitsIncrementalTaskAllocationNode::setTaskTraitAllocation(
            const std::shared_ptr<const TaskTraitAllocation>& task_trait_allocations) const
    {
        m_task_trait_allocations = task_trait_allocations;
    }

    const std::shared_ptr<const TaskTraitAllocation> TraitsIncrementalTaskAllocationNode::taskTraitAllocation() const
    {
        return m_task_trait_allocations;
    }

}  // namespace traits