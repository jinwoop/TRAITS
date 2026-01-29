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
#include "traits/task_allocation/task_allocation_math.hpp"

// Local
#include "traits/task.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/task_allocation/itags/robot_traits_matrix_reduction.hpp"

namespace traits
{
    Eigen::MatrixXf maxTeamTraitsMatrix(const std::vector<std::shared_ptr<const Robot>>& robots)
    {
        if(robots.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = robots.front()->species()->traits_max().size();

        Eigen::MatrixXf rv(robots.size(), num_traits);
        unsigned int row_nr = 0;
        for(const auto& robot : robots)
        {
            rv.row(row_nr++) = robot->initial_traits();
        }
        return rv;
    }

    Eigen::MatrixXf maxTeamTraitRatesMatrix(const std::vector<std::shared_ptr<const Robot>>& robots)
    {
        if(robots.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = robots.front()->species()->trait_rates_max().size();

        Eigen::MatrixXf rv(robots.size(), num_traits);
        unsigned int row_nr = 0;
        for(const auto& robot : robots)
        {
            rv.row(row_nr++) = robot->species()->trait_rates_max();
        }
        return rv;
    }

    float TraitAndRatesMistmatch(const Eigen::MatrixXf& allocation,
                                     const Eigen::MatrixXf& desired_traits_matrix,
                                     const Eigen::MatrixXf& desired_trait_rates_matrix,
                                     const Eigen::MatrixXf& max_team_traits_matrix,
                                     const Eigen::MatrixXf& max_team_trait_rates_matrix,
                                     const std::vector<std::shared_ptr<const Task>>& tasks,
                                     float gamma) {
            int num_tasks = desired_traits_matrix.rows();
            int num_traits = desired_traits_matrix.cols();

            Eigen::MatrixXf allocated_traits = Eigen::MatrixXf::Zero(num_tasks, num_traits);
            Eigen::MatrixXf allocated_trait_rates = Eigen::MatrixXf::Zero(num_tasks, num_traits);
            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                // Create the allocated traits matrix for the task
                const Eigen::VectorXf& task_allocation_vector = allocation.row(task_nr);
                Eigen::VectorXi is_selected                   = (task_allocation_vector.array() > 0.5).cast<int>();
                Eigen::MatrixXf allocated_traits_matrix(is_selected.sum(), max_team_traits_matrix.cols());
                Eigen::MatrixXf allocated_trait_rates_matrix(is_selected.sum(), max_team_trait_rates_matrix.cols());
                unsigned int row_nr = 0;
                for(unsigned int i = 0, i_end = max_team_traits_matrix.rows(); i < i_end; ++i)
                {
                    if(is_selected[i])
                    {
                        allocated_traits_matrix.row(row_nr) = max_team_traits_matrix.row(i);
                        allocated_trait_rates_matrix.row(row_nr++) = max_team_trait_rates_matrix.row(i);
                    }
                }

                for(unsigned int trait_nr = 0; trait_nr < num_traits and allocated_traits_matrix.rows() > 0; ++trait_nr)
                {
                    if (tasks[task_nr]->traits_aggregatable()[trait_nr]) {
                        allocated_traits(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).sum();
                        allocated_trait_rates(task_nr, trait_nr) = allocated_trait_rates_matrix.col(trait_nr).sum();
                    }
                    else {
                        allocated_traits(task_nr, trait_nr) = allocated_traits_matrix.col(trait_nr).minCoeff();
                        allocated_trait_rates(task_nr, trait_nr) = allocated_trait_rates_matrix.col(trait_nr).minCoeff();
                    }
                }
            }

            float rv = 0.0f;
            // gamma * A * Q
            Eigen::MatrixXf traits_diff = desired_traits_matrix - allocated_traits;
            Eigen::MatrixXf traits_diff_positive = (traits_diff.array() < 0).select(0, traits_diff);
            rv += gamma * traits_diff_positive.sum() / desired_traits_matrix.sum();

            // A * Qdot
            Eigen::MatrixXf trait_rates_diff = desired_trait_rates_matrix - allocated_trait_rates;
            Eigen::MatrixXf trait_rates_diff_positive = (trait_rates_diff.array() < 0).select(0, trait_rates_diff);
            rv += (1.0f - gamma) * trait_rates_diff_positive.sum() / desired_trait_rates_matrix.sum();

            // E(A) = Y - A * Q
            return rv;
    }

    Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                        const std::vector<unsigned int>& plan_task_indicies)
    {
        if(tasks.empty() || plan_task_indicies.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = tasks.front()->desiredTraits().size();

        Eigen::MatrixXf rv(plan_task_indicies.size(), num_traits);
        unsigned int row_nr = 0;
        for(const unsigned int index: plan_task_indicies)
        {
            rv.row(row_nr++) = tasks[index]->desiredTraits();
        }
        return rv;
    }

    Eigen::MatrixXf desiredTraitRatesMatrix(const std::vector<std::shared_ptr<const Task>>& tasks,
                                            const std::vector<unsigned int>& plan_task_indicies)
    {
        if(tasks.empty() || plan_task_indicies.empty())
        {
            return Eigen::MatrixXf();
        }

        const unsigned int num_traits = tasks.front()->minimum_trait_rates().size();

        Eigen::MatrixXf rv(plan_task_indicies.size(), num_traits);
        unsigned int row_nr = 0;
        for(const unsigned int index: plan_task_indicies)
        {
            rv.row(row_nr++) = tasks[index]->minimum_trait_rates();
        }
        return rv;
    }

    Eigen::MatrixXf desiredTraitsMatrix(const std::vector<std::shared_ptr<const Task>>& tasks)
    {
        if(tasks.empty())
        {
            return Eigen::MatrixXf();
        }
        const unsigned int num_traits = tasks.front()->desiredTraits().size();
        Eigen::MatrixXf rv(tasks.size(), num_traits);
        unsigned int row_nr = 0;
        for(const std::shared_ptr<const Task>& task: tasks)
        {
            rv.row(row_nr++) = task->desiredTraits();
        }
        return rv;
    }

    Eigen::MatrixXf allocatedTraitsMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                          const Eigen::MatrixXf& allocation,
                                          const Eigen::MatrixXf& robot_traits_matrix)
    {
        return robot_traits_matrix_reduction.reduce(allocation, robot_traits_matrix);
    }

    Eigen::MatrixXf traitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                         const Eigen::MatrixXf& allocation,
                                         const Eigen::MatrixXf& desired_traits_matrix,
                                         const Eigen::MatrixXf& robot_traits_matrix)
    {
        // A * Q
        Eigen::MatrixXf allocated_traits_matrix =
                allocatedTraitsMatrix(robot_traits_matrix_reduction, allocation, robot_traits_matrix);

        // E(A) = Y - A * Q
        return desired_traits_matrix - allocated_traits_matrix;
    }

    Eigen::MatrixXf positiveOnlyTraitsMismatchMatrix(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                                                     const Eigen::MatrixXf& allocation,
                                                     const Eigen::MatrixXf& desired_traits_matrix,
                                                     const Eigen::MatrixXf& robot_traits_matrix)
    {
        Eigen::MatrixXf traits_mismatch_matrix =
                traitsMismatchMatrix(robot_traits_matrix_reduction, allocation, desired_traits_matrix, robot_traits_matrix);

        return (traits_mismatch_matrix.array() < 0).select(0, traits_mismatch_matrix);
    }

    float traitsMismatchError(const RobotTraitsMatrixReduction& robot_traits_matrix_reduction,
                              const Eigen::MatrixXf& allocation,
                              const Eigen::MatrixXf& desired_traits_matrix,
                              const Eigen::MatrixXf& robot_traits_matrix)
    {
        return positiveOnlyTraitsMismatchMatrix(robot_traits_matrix_reduction,
                                                allocation,
                                                desired_traits_matrix,
                                                robot_traits_matrix)
                .sum();
    }

    std::set<std::pair<unsigned int, unsigned int>> computeMutexConstraints(const Eigen::MatrixXf& allocation)
    {
        if(allocation.isZero())
        {
            return std::set<std::pair<unsigned int, unsigned int>>();
        }

        std::set<std::pair<unsigned int, unsigned int>> mutex_constraints;
        const unsigned int num_tasks  = allocation.rows();
        const unsigned int num_robots = allocation.cols();
        for(unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
        {
            std::vector<unsigned int> allocated_tasks;
            for(unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
            {
                if(allocation(task_nr, robot_nr))
                {
                    allocated_tasks.push_back(task_nr);
                }
            }
            if(!allocated_tasks.empty())
            {
                for(unsigned int task_i_index = 0, num_assigned_tasks = allocated_tasks.size();
                    task_i_index < num_assigned_tasks;
                    ++task_i_index)
                {
                    const unsigned int task_i = allocated_tasks[task_i_index];
                    for(unsigned int task_j_index = task_i_index + 1; task_j_index < num_assigned_tasks; ++task_j_index)
                    {
                        const unsigned int task_j = allocated_tasks[task_j_index];
                        mutex_constraints.insert({task_i, task_j});
                    }
                }
            }
        }
        return mutex_constraints;
    }

    /*
     * If there are two constraints (1, 2), (2, 3),
     * then a new transitive constraint is (1, 3).
     */
    std::set<std::pair<unsigned int, unsigned int>> addPrecedenceTransitiveConstraints(
            std::set<std::pair<unsigned int, unsigned int>> ordering_constraints)
    {
        unsigned int num_ordering_constraints = ordering_constraints.size();
        while(true)
        {
            std::set<std::pair<unsigned int, unsigned int>> temp;
            for(const std::pair<unsigned int, unsigned int>& constraint1: ordering_constraints)
            {
                temp.insert(constraint1);
                for(const std::pair<unsigned int, unsigned int>& constraint2: ordering_constraints)
                {
                    if(constraint1.second == constraint2.first)
                    {
                        temp.insert(std::pair(constraint1.first, constraint2.second));
                    }
                }
            }
            ordering_constraints = std::move(temp);
            if(num_ordering_constraints == ordering_constraints.size())
            {
                break;
            }
            num_ordering_constraints = ordering_constraints.size();
        }
        return ordering_constraints;
    }


    float necessaryConditionViolation(const Eigen::MatrixXf& allocation,
                                     const Eigen::MatrixXf& desired_traits_matrix,
                                     const Eigen::MatrixXf& desired_trait_rates_matrix,
                                     const Eigen::MatrixXf& max_team_traits_matrix,
                                     const Eigen::MatrixXf& max_team_trait_rates_matrix,
                                     const std::vector<std::shared_ptr<const Task>>& tasks) {

        int num_robots = allocation.cols();
        int num_tasks = allocation.rows();
        int num_traits = desired_traits_matrix.cols();

        Eigen::MatrixXf selected_desired_traits = Eigen::MatrixXf::Zero(num_tasks, num_traits);

        Eigen::VectorXi allocated_robots = Eigen::VectorXi::Zero(num_robots);
        for (unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
        {
            // if robot is assigned to at least one task
            if (allocation.col(robot_nr).sum() > 0.5) {
                allocated_robots( robot_nr) = 1;
            }
        }
        // the number of row of this matrix is at max all robots, but generally smaller
        Eigen::MatrixXf allocated_traits_matrix = Eigen::MatrixXf::Zero(allocated_robots.sum(), num_traits);
        unsigned int assigned_robot_nr = 0;
        for (unsigned int robot_nr = 0; robot_nr < num_robots; ++robot_nr)
        {
           // if robot is assigned to at least one task
           if (allocation.col(robot_nr).sum() > 0.5) {
               allocated_traits_matrix.row(assigned_robot_nr++) = max_team_traits_matrix.row(robot_nr);
           }
        }

        for (unsigned int task_nr = 0; task_nr < num_tasks; ++task_nr)
        {
            for(unsigned int trait_nr = 0; trait_nr < num_traits and allocated_traits_matrix.rows() > 0; ++trait_nr)
            {
                if (tasks[task_nr]->traits_aggregatable()[trait_nr]) {
                    selected_desired_traits(task_nr, trait_nr) += desired_traits_matrix(task_nr, trait_nr);
                }
                else {
                    selected_desired_traits(task_nr, trait_nr) =
                            std::max(selected_desired_traits(task_nr, trait_nr), desired_traits_matrix(task_nr, trait_nr));
                }
            }
        }

        Eigen::MatrixXf total_traits_missing = Eigen::MatrixXf::Zero(1, max_team_traits_matrix.cols());
        for(unsigned int trait_nr = 0; trait_nr < num_traits; ++trait_nr)
        {
            // every task have same trait type per index
            if (tasks[0]->traits_aggregatable()[trait_nr]) {
                if (allocated_traits_matrix.rows() > 0)
                {
                    total_traits_missing(trait_nr) =
                            selected_desired_traits.col(trait_nr).sum() - allocated_traits_matrix.col(trait_nr).sum();
                } else {
                    total_traits_missing(trait_nr) = selected_desired_traits.col(trait_nr).sum();
                }
            } else {
                if (allocated_traits_matrix.rows() > 0) {
                    // maximum trait required by task subtract least capable robot checking.
                    total_traits_missing(trait_nr) =
                            selected_desired_traits.col(trait_nr).maxCoeff() -
                            allocated_traits_matrix.col(trait_nr).minCoeff();
                } else {
                    total_traits_missing(trait_nr) = selected_desired_traits.col(trait_nr).maxCoeff();
                }
            }
        }

        return total_traits_missing.sum();
    }
}  // namespace traits
