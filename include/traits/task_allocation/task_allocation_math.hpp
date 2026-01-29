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
#include <memory>
#include <set>
// External
#include <eigen3/Eigen/Core>

namespace traits
{
    // Forward Declarations
    class RobotTraitsMatrixReduction;
    class Task;
    class Robot;
    class TraitsProblemInputs;

    [[nodiscard]] Eigen::MatrixXf maxTeamTraitsMatrix(const std::vector<std::shared_ptr<const Robot>>& robots);

    [[nodiscard]] Eigen::MatrixXf maxTeamTraitRatesMatrix(const std::vector<std::shared_ptr<const Robot>>& robots);

    [[nodiscard]] float TraitAndRatesMistmatch(const Eigen::MatrixXf& allocation,
                                                   const Eigen::MatrixXf& desired_traits_matrix,
                                                   const Eigen::MatrixXf& desired_trait_rates_matrix,
                                                   const Eigen::MatrixXf& max_team_traits_matrix,
                                                   const Eigen::MatrixXf& max_team_trait_rates_matrix,
                                                   const std::vector<std::shared_ptr<const Task>>& tasks,
                                                   float gamma);

    [[nodiscard]] Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                                      const std::vector<unsigned int>& plan_task_indicies);

    [[nodiscard]] Eigen::MatrixXf desiredTraitRatesMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                                      const std::vector<unsigned int>& plan_task_indicies);


    [[nodiscard]] Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks);

    [[nodiscard]] Eigen::MatrixXf teamTraitsMatrix(const std::vector<std::shared_ptr<const Robot>>& robots);

    //! \returns The allocated traits matrix
    [[nodiscard]] Eigen::MatrixXf allocatedTraitsMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                        const Eigen::MatrixXf& allocation,
                                                        const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits mismatch matrix
    [[nodiscard]] Eigen::MatrixXf traitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                       const Eigen::MatrixXf& allocation,
                                                       const Eigen::MatrixXf& desired_traits_matrix,
                                                       const Eigen::MatrixXf& robot_traits_matrix);

    [[nodiscard]] float necessaryConditionViolation(const Eigen::MatrixXf& allocation,
                                                    const Eigen::MatrixXf& desired_traits_matrix,
                                                    const Eigen::MatrixXf& desired_trait_rates_matrix,
                                                    const Eigen::MatrixXf& max_team_traits_matrix,
                                                    const Eigen::MatrixXf& max_team_trait_rates_matrix,
                                                    const std::vector<std::shared_ptr<const Task>>& tasks);

    //! \returns The traits mismatch matrix with all negative values removed
    [[nodiscard]] Eigen::MatrixXf positiveOnlyTraitsMismatchMatrix(
        const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
        const Eigen::MatrixXf& allocation,
        const Eigen::MatrixXf& desired_traits_matrix,
        const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns The traits mismatch error
    [[nodiscard]] float traitsMismatchError(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                           const Eigen::MatrixXf& allocation,
                                           const Eigen::MatrixXf& desired_traits_matrix,
                                           const Eigen::MatrixXf& robot_traits_matrix);

    [[nodiscard]] float traitsMismatchErrorTask(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                const Eigen::MatrixXf& allocation,
                                                const Eigen::MatrixXf& desired_traits_matrix,
                                                const Eigen::MatrixXf& robot_traits_matrix,
                                                unsigned int task_nr);

    [[nodiscard]] float traitsMismatchErrorStochastic(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                      const Eigen::MatrixXf& allocation,
                                                      const Eigen::MatrixXf& desired_traits_matrix,
                                                      const Eigen::MatrixXf& robot_traits_matrix);

    //! \returns A set of the mutex constraints for an allocation
    std::set<std::pair<unsigned int, unsigned int>> computeMutexConstraints(const Eigen::MatrixXf& allocation);

    /*!
     * Updates the set by adding transitive constraints (0,1) ^ (1,2) -> (0, 2)
     *
     * \returns The updated set
     */
    std::set<std::pair<unsigned int, unsigned int>> addPrecedenceTransitiveConstraints(
        std::set<std::pair<unsigned int, unsigned int>> ordering_constraints);
}  // namespace traits