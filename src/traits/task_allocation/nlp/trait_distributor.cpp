/* Modeling and Optimizing the Provisioning of Exhaustible Capabilities
 * for Simultaneous Task Allocation and Scheduling
 *
 * Author: Jinwoo Park
 *
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
#include "traits/task_allocation/nlp/trait_distributor.hpp"

#include <cmath>

#include "fmt/core.h"

#include "traits/robot.hpp"
#include "traits/species.hpp"
#include "traits/task.hpp"
#include "traits/common/nlp/nlp_failure_reason.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/task_allocation/nlp/trait_distributor_result.hpp"
#include "traits/task_allocation/nlp/task_trait_allocation.hpp"

namespace traits {

    TraitDistributor::TraitDistributor(const Eigen::MatrixXf& allocation,
                                         const std::shared_ptr<const TraitsProblemInputs>& problem_inputs)
            : NlpSolverBase(0)
            , m_allocation(allocation)
            , m_traits_problem_inputs(problem_inputs)
            , numTasks(problem_inputs->numberOfTasks())
            , numRobots(problem_inputs->numberOfRobots())
            , numTraits(problem_inputs->numberOfTraits())
            , m_gamma(m_traits_problem_inputs->gamma())
            , m_Esum(m_traits_problem_inputs->desiredTraitsMatrix().sum())
            , m_Edotsum(m_traits_problem_inputs->desiredTraitRatesMatrix().sum())
            /****************************** GUROBI VARIABLES ******************************/
            // TRAITS
            , m_total_traits_unfulfilled(nullptr)
            , m_total_trait_rates_unfulfilled(nullptr)
            , m_task_traits_unfulfilled(problem_inputs->numberOfTasks(),
                                        std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(),
                                                                             nullptr))
            , m_task_trait_rates_unfulfilled(problem_inputs->numberOfTasks(),
                                             std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(),
                                                                                  nullptr))
            , m_trait_variables(problem_inputs->numberOfTasks(),
                                std::vector<std::vector<std::shared_ptr<GRBVar> > >(
                                        problem_inputs->numberOfRobots(),
                                        std::vector<std::shared_ptr<GRBVar>>(
                                                problem_inputs->numberOfTraits(), nullptr)
                                        )
                                )
            , m_trait_rate_variables(problem_inputs->numberOfTasks(),
                                     std::vector<std::vector<std::shared_ptr<GRBVar> > >(
                                             problem_inputs->numberOfRobots(),
                                             std::vector<std::shared_ptr<GRBVar>>(
                                                     problem_inputs->numberOfTraits(), nullptr)
                                             )
                                    )
            , m_robot_trait_consumption_sum_or_max(problem_inputs->numberOfRobots(),
                                            std::vector<std::shared_ptr<GRBLinExpr>>(problem_inputs->numberOfTraits(),
                                                                                     nullptr))
            , m_coalition_traits_sum_expr(problem_inputs->numberOfTasks(), // note: very critical mistake
                                          std::vector<std::shared_ptr<GRBLinExpr>>(problem_inputs->numberOfTraits(),
                                                                                   nullptr)) // note: even crazy critical
            , m_coalition_trait_rates_avg_var(problem_inputs->numberOfTasks(),
                                              std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(),
                                                                                   nullptr))
            // TIME
            , m_total_task_durations(nullptr)
            , m_task_durations(problem_inputs->numberOfTasks(), nullptr)
            , m_task_dynamic_durations(problem_inputs->numberOfTasks(), nullptr)
            , m_task_robot_trait_durations(problem_inputs->numberOfTasks(),
                               std::vector<std::vector<std::shared_ptr<GRBVar>>>(problem_inputs->numberOfRobots(),
                                           std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(),
                                                                                nullptr)))
            , m_task_trait_durations(problem_inputs->numberOfTasks(),
                                     std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(), nullptr))
            , m_task_intra_transition_durations(problem_inputs->numberOfTasks(), nullptr)
            , m_robot_inter_transition_velocities(problem_inputs->numberOfRobots(), nullptr)
            , m_robot_inter_transition_ub_durations(problem_inputs->numberOfRobots(), nullptr)
            // VELOCITIES
            , m_task_coalition_velocities(problem_inputs->numberOfTasks(), nullptr)
            // CURRENT
            , m_task_robot_trait_currents(problem_inputs->numberOfTasks(),
                                          std::vector<std::vector<std::shared_ptr<GRBVar>>>(problem_inputs->numberOfRobots(),
                                                                                            std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(), nullptr)))
            , m_task_robot_trait_rate_currents(problem_inputs->numberOfTasks(),
                                               std::vector<std::vector<std::shared_ptr<GRBVar>>>(problem_inputs->numberOfRobots(),
                                                                                                 std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfTraits(), nullptr)))
            , m_robot_inter_transition_currents_LExpr(problem_inputs->numberOfRobots(), nullptr)
            , m_robot_inter_transition_currents_var(problem_inputs->numberOfRobots(), nullptr)
            , m_robot_inter_transition_currents(problem_inputs->numberOfRobots(), nullptr)
            , m_task_robot_currents_expr(problem_inputs->numberOfTasks(),
                                         std::vector<std::shared_ptr<GRBLinExpr>>(problem_inputs->numberOfRobots(), nullptr))
            , m_task_robot_currents_expr_var(problem_inputs->numberOfTasks(),
                                        std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfRobots(), nullptr))
            , m_task_robot_currents_var(problem_inputs->numberOfTasks(),
                                        std::vector<std::shared_ptr<GRBVar>>(problem_inputs->numberOfRobots(), nullptr))
            , m_robot_battery_consumption_expr(problem_inputs->numberOfRobots(), nullptr)

            /****************************** RESULT VARIABLES ******************************/
            // TRAIT
            , m_result_trait_deficiencies(std::numeric_limits<float>::infinity())
            , m_result_trait_rate_deficiencies(std::numeric_limits<float>::infinity())
            , m_result_robot_task_traits(problem_inputs->numberOfTasks(),
                                         std::vector<std::vector<float>>(problem_inputs->numberOfRobots(),
                                                                         std::vector<float>(problem_inputs->numberOfTraits(), 0.0)))
            , m_result_robot_task_trait_rates(problem_inputs->numberOfTasks(),
                                              std::vector<std::vector<float>>(problem_inputs->numberOfRobots(),
                                                                              std::vector<float>(problem_inputs->numberOfTraits(), 0.0)))
            , m_result_coalition_traits(problem_inputs->numberOfTasks(),
                                        std::vector<float>(problem_inputs->numberOfTraits(),
                                                           std::numeric_limits<float>::infinity()))
            , m_result_coalition_trait_rates(problem_inputs->numberOfTasks(),
                                             std::vector<float>(problem_inputs->numberOfTraits(),
                                                                std::numeric_limits<float>::infinity()))
            // TIME
            , m_result_task_dynamic_durations(problem_inputs->numberOfTasks(), 0.0)
            , m_result_task_total_durations(problem_inputs->numberOfTasks(), 0.0)
            , m_result_task_intra_transitions(problem_inputs->numberOfTasks(), 0.0)
            , m_result_task_trait_durations(problem_inputs->numberOfTasks(),
                                            std::vector<float>(problem_inputs->numberOfTraits(), 0.0))
            , m_result_robot_battery_consumptions(problem_inputs->numberOfRobots(), 0.0)
            , m_result_robot_inter_task_transition_velocities(problem_inputs->numberOfRobots(), 0.0)
    {}

    std::shared_ptr<const TraitDistributorResult> TraitDistributor::solve()
    {
        TimerRunner timer_runner(constants::k_task_distribution_time);
        return computeTraitDistribution();
    }


    /*
     * This function is to extract NLP results into reader friendly format.
     */
    std::shared_ptr<const TaskTraitAllocation> TraitDistributor::createTaskTraitAllocations() {

        // instantiate these variables within this scope only
        std::vector<float> task_times;

        /*
         * about Tasks
         */
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            m_result_task_dynamic_durations[task_nr] = (m_task_dynamic_durations[task_nr] != nullptr) ?
                                               m_task_dynamic_durations[task_nr]->get(GRB_DoubleAttr_X) : 0.0;

            m_result_task_total_durations[task_nr] = (m_task_durations[task_nr] != nullptr) ?
                                               m_task_durations[task_nr]->get(GRB_DoubleAttr_X) : 0.0;

            task_times.push_back(m_result_task_total_durations[task_nr]);

            // set it in task object
            m_traits_problem_inputs->task(task_nr)->setTotalDuration(m_result_task_total_durations[task_nr]);


            m_result_task_intra_transitions[task_nr] = (m_task_intra_transition_durations[task_nr] != nullptr) ?
                                                     m_task_intra_transition_durations[task_nr]->get(GRB_DoubleAttr_X) : 0.0;
        }

        /*
         * about Robots
         */
        for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr)
        {
            if (m_robot_battery_consumption_expr[robot_nr] != nullptr) {
                m_traits_problem_inputs->robot(robot_nr)->set_inter_transition_velocity(
                        m_robot_inter_transition_velocities[robot_nr]->get(GRB_DoubleAttr_X));

                m_result_robot_inter_task_transition_velocities[robot_nr] =
                        m_robot_inter_transition_velocities[robot_nr]->get(GRB_DoubleAttr_X);

                m_result_robot_battery_consumptions[robot_nr] =
                        m_robot_battery_consumption_expr[robot_nr]->getValue();
            } else {
                // readability: already zeros
            }
        }

        for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
            for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {

                // if no robot assigned, then all traits remains
                if (!taskHasAssignedRobot(task_nr) || !taskTraitRequired(task_nr, trait_nr)) {
                    m_result_coalition_traits[task_nr][trait_nr] = 0.0;
                    m_result_coalition_trait_rates[task_nr][trait_nr] = 0.0;
                    continue;
                }

                /*
                 * Task Traits aggregatable
                 */
                if (taskTraitAllowsCoalition(task_nr, trait_nr)) {
                    m_result_coalition_traits[task_nr][trait_nr] = m_coalition_traits_sum_expr[task_nr][trait_nr]->getValue();

                    // handles both provisioning and non-provisioning traits
                    m_result_coalition_trait_rates[task_nr][trait_nr] =
                            (m_coalition_trait_rates_avg_var[task_nr][trait_nr] != nullptr) ?
                            m_coalition_trait_rates_avg_var[task_nr][trait_nr]->get(GRB_DoubleAttr_X) : 0.0;
                } else {   // task traits individual (MIN)
                    for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
                        // if there is at least one robot assigned to the task
                        // but this robot is not assigned to this task
                        if (!robotHasAssignedTask(robot_nr, task_nr)) {
                            continue;
                        }

                        // following ONLY considers actually assigned robots.
                        // when trait is not aggregatable, then the robot with minimum trait will be the bottleneck for
                        // the requirement.

                        // fixed -- above keeps having positive apr even though assigned robots have sufficient traits
                        if (m_trait_variables[task_nr][robot_nr][trait_nr] != nullptr) {
                            m_result_coalition_traits[task_nr][trait_nr] =
                                    std::min(m_result_coalition_traits[task_nr][trait_nr],
                                             (float) m_trait_variables[task_nr][robot_nr][trait_nr]->get(
                                                     GRB_DoubleAttr_X));
                        }

                        // if non-provisionable, then the rate is 0.0
                        m_result_coalition_trait_rates[task_nr][trait_nr] =
                                std::min(
                                        m_result_coalition_trait_rates[task_nr][trait_nr],
                                        (m_trait_rate_variables[task_nr][robot_nr][trait_nr] != nullptr)
                                        ? (float) m_trait_rate_variables[task_nr][robot_nr][trait_nr]->get(
                                                GRB_DoubleAttr_X)
                                        : 0.0f
                                );

                    } //end for
                } // end else -- non coalitionable
            }
        }

        for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
            for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
                // following applies to all tasks
                m_result_task_trait_durations[task_nr][trait_nr] =
                        (m_task_trait_durations[task_nr][trait_nr] != nullptr) ?
                        m_task_trait_durations[task_nr][trait_nr]->get(GRB_DoubleAttr_X) : 0.0f;

                for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
                    m_result_robot_task_traits[task_nr][robot_nr][trait_nr] =
                            (m_trait_variables[task_nr][robot_nr][trait_nr] != nullptr) ?
                            m_trait_variables[task_nr][robot_nr][trait_nr]->get(GRB_DoubleAttr_X) : 0.0f;

                    m_result_robot_task_trait_rates[task_nr][robot_nr][trait_nr] =
                            (m_trait_rate_variables[task_nr][robot_nr][trait_nr] != nullptr) ?
                            m_trait_rate_variables[task_nr][robot_nr][trait_nr]->get(GRB_DoubleAttr_X) : 0.0f;
                }
            }
        }

        m_result_trait_deficiencies = 0.0;   // was asking for vairable w/o any constraints (i.e. 0)
        m_result_trait_rate_deficiencies = 0.0;
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                m_result_trait_deficiencies += std::max(0.0f,
                        m_traits_problem_inputs->task(task_nr)->desiredTraits()(trait_nr) -
                        m_result_coalition_traits[task_nr][trait_nr]);
                m_result_trait_rate_deficiencies += std::max(0.0f,
                        m_traits_problem_inputs->task(task_nr)->minimum_trait_rates()(trait_nr) -
                        m_result_coalition_trait_rates[task_nr][trait_nr]);
            }
        }

        return std::make_shared<TaskTraitAllocation>(m_result_trait_deficiencies,
                                                     m_result_trait_rate_deficiencies,
                                                     m_result_robot_task_traits,
                                                     m_result_robot_task_trait_rates,
                                                     m_result_coalition_traits,
                                                     m_result_coalition_trait_rates,
                                                     m_result_task_dynamic_durations,
                                                     m_result_task_total_durations,
                                                     m_result_task_intra_transitions,
                                                     m_result_task_trait_durations,
                                                     m_result_robot_inter_task_transition_velocities,
                                                     m_result_robot_battery_consumptions);
    }

    /*
     * This function actually computes the NLP -- most high-level part in NLP
     */
    std::shared_ptr<const TraitDistributorResult> TraitDistributor::computeTraitDistribution() {
        std::shared_ptr<NlpSolverResult> result = solveNlp(m_traits_problem_inputs->traitDistributionParameters());
        if (result->failure()) {
            return std::make_shared<TraitDistributorResult>(result->failureReason());
        }

        if(auto task_trait_allocations = createTaskTraitAllocations(); task_trait_allocations)
        {
            return std::make_shared<TraitDistributorResult>(task_trait_allocations);
        }
        return std::make_shared<TraitDistributorResult>(std::make_shared<NlpFailureReason>());
    }

    std::shared_ptr<const FailureReason> TraitDistributor::setupData()
    {
        // All done in nlp_solver_base.cpp
        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitDistributor::createObjective(GRBModel& model)
    {
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        int obj_index = -1;
        int priority = 100;

        if (m_Edotsum == 0.0) {
            model.setObjectiveN((m_gamma / m_Esum) * *m_total_traits_unfulfilled, ++obj_index, priority);
        } else {
            model.setObjectiveN((m_gamma / m_Esum) * *m_total_traits_unfulfilled
                                + ((1.0f - m_gamma) / m_Edotsum) * *m_total_trait_rates_unfulfilled,
                                ++obj_index, priority);
        }

        --priority;
        model.setObjectiveN(*m_total_task_durations, ++obj_index, priority);

        --priority;
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                if (taskTraitRequired(task_nr, trait_nr) && taskHasAssignedRobot(task_nr)) {
                    model.setObjectiveN(*m_task_traits_unfulfilled[task_nr][trait_nr], ++obj_index, priority);
                    if (taskTraitRateRequired(task_nr, trait_nr)) {
                        model.setObjectiveN(*m_task_trait_rates_unfulfilled[task_nr][trait_nr], ++obj_index, priority);
                    }
                }
            }
        }

        --priority;
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                if (taskTraitRequired(task_nr, trait_nr) \
                    && taskHasAssignedRobot(task_nr) \
                    && taskTraitRateRequired(task_nr, trait_nr)) {
                    // note: this also maximizes the trait rates
                    model.setObjectiveN(*m_task_trait_durations[task_nr][trait_nr], ++obj_index, priority);
                }
            }
        }

        --priority;
        for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr)
        {
            if (robotIsAssigned(robot_nr)) {
                model.setObjectiveN(-*m_robot_inter_transition_velocities[robot_nr], ++obj_index, priority);
            }
        }

        return nullptr;
    }

    std::shared_ptr<const FailureReason> TraitDistributor::createVariables(GRBModel& model)
    {
        m_total_traits_unfulfilled = std::make_shared<GRBLinExpr>(0.0);         // E
        m_total_trait_rates_unfulfilled = std::make_shared<GRBLinExpr>(0.0);    // Edot
        m_total_task_durations = std::make_shared<GRBLinExpr>(0.0);

        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            if (!taskHasAssignedRobot(task_nr)) {
                continue;
            }

            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                // if task does not require this trait, don't create variable
                if (!taskTraitRequired(task_nr, trait_nr)) {
                    continue;
                }

                m_task_traits_unfulfilled[task_nr][trait_nr]  // already counting for only positives
                        = std::make_shared<GRBVar>(model.addVar(0.0,  // instead of 0.0 -- numerical instability
                                                                GRB_INFINITY,
                                                                0.0,
                                                                GRB_CONTINUOUS,
                                                                fmt::format("E_k{:d}u{:d}",
                                                                            task_nr,
                                                                            trait_nr)));

                m_task_trait_rates_unfulfilled[task_nr][trait_nr]  // already counting for only positives
                        = std::make_shared<GRBVar>(model.addVar(0.0,  // instead of 0.0 -- numerical instability
                                                                GRB_INFINITY,
                                                                0.0,
                                                                GRB_CONTINUOUS,
                                                                fmt::format("Edot_k{:d}u{:d}",
                                                                            task_nr,
                                                                            trait_nr)));

                if (taskTraitAllowsCoalition(task_nr, trait_nr))
                {
                    // Trait Sum
                    m_coalition_traits_sum_expr[task_nr][trait_nr] = std::make_shared<GRBLinExpr>(0.0);

                    // Average Coalition Trait Rates
                    if (taskTraitProvisionable(task_nr, trait_nr))
                    {
                        m_coalition_trait_rates_avg_var[task_nr][trait_nr]
                                = std::make_shared<GRBVar>(model.addVar(0.0,
                                                                        GRB_INFINITY,
                                                                        0.0,
                                                                        GRB_CONTINUOUS,
                                                                        fmt::format("Avg_Yd_k{:d}u{:d}",
                                                                                    task_nr,
                                                                                    trait_nr)));
                    }
                }  // end if
            }  // end for trait_nr
        }  // end for task_nr


        float makespan_ub = m_traits_problem_inputs->scheduleWorstMakespan();
        // Task Trait Duration Variable Initializations
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            if (!taskHasAssignedRobot(task_nr)) {
                continue;
            }

            m_task_dynamic_durations[task_nr] =
                    std::make_shared<GRBVar>(model.addVar(0.0,
                                                          makespan_ub,
                                                          0.0,
                                                          GRB_CONTINUOUS,
                                                          fmt::format("t_q_k{:d}", task_nr)));

            m_task_intra_transition_durations[task_nr] =
                    std::make_shared<GRBVar>(model.addVar(0.0,
                                                          makespan_ub,
                                                          0.0,
                                                          GRB_CONTINUOUS,
                                                          fmt::format("phi_intra_k{:d}", task_nr)));

            m_task_durations[task_nr] =
                    std::make_shared<GRBVar>(model.addVar(0.0,
                                                          makespan_ub,
                                                          0.0,
                                                          GRB_CONTINUOUS,
                                                          fmt::format("t_k{:d}", task_nr)));


            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr)
            {
                // if task does not require this trait, don't create variable
                if (!taskTraitRequired(task_nr, trait_nr) || !taskTraitRateRequired(task_nr, trait_nr)) {
                    continue;
                }

                m_task_trait_durations[task_nr][trait_nr] =
                        std::make_shared<GRBVar>(model.addVar(0.0,
                                                              makespan_ub,
                                                              0.0,
                                                              GRB_CONTINUOUS,
                                                              fmt::format("t_k{:d}u{:d}",
                                                                          task_nr, trait_nr)));
            } // end for trait_nr
        } // end for task_nr



        // Task Intra-Transition Velocities
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
            if (!taskHasAssignedRobot(task_nr)) {
                continue;
            }

            float coalition_max_velocity = std::numeric_limits<float>::infinity();

            for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
                if (!robotHasAssignedTask(robot_nr, task_nr)) {
                    continue;
                }
                coalition_max_velocity = std::min(coalition_max_velocity,
                                                  m_traits_problem_inputs->robot(robot_nr)->species()->max_speed());
            }

            m_task_coalition_velocities[task_nr]
                    = std::make_shared<GRBVar>(model.addVar(0.0,
                                                            double(coalition_max_velocity),
                                                            0.0,
                                                            GRB_CONTINUOUS,
                                                            fmt::format("V_intra_k{:d}", task_nr)));
        } // ----- Task Related: END ----- //



        // ----- Robot Related: START ----- //
        for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
            if (!robotIsAssigned(robot_nr))
            {
                continue;
            }

            /******************** Velcity and Time ********************/
            m_robot_inter_transition_velocities[robot_nr]
                    = std::make_shared<GRBVar>(
                    model.addVar(0.0,
                                 double(m_traits_problem_inputs->robot(robot_nr)->species()->max_speed()),
                                 0.0,
                                 GRB_CONTINUOUS,
                                 fmt::format("V_inter_n{:d}", robot_nr)));


            m_robot_inter_transition_ub_durations[robot_nr]
                    = std::make_shared<GRBVar>(
                    model.addVar(0.0,
                                 GRB_INFINITY,
                                 0.0,
                                 GRB_CONTINUOUS,
                                 fmt::format("phi_inter_n{:d}", robot_nr)));

            /******************** Currents ********************/
            m_robot_battery_consumption_expr[robot_nr] = std::make_shared<GRBQuadExpr>(0.0);

            m_robot_inter_transition_currents_LExpr[robot_nr] = std::make_shared<GRBLinExpr>(0.0);

            m_robot_inter_transition_currents_var[robot_nr]
                    = std::make_shared<GRBVar>(
                    model.addVar(0.0,
                                 double(m_traits_problem_inputs->robot(robot_nr)->species()->max_possible_current()),
                                 0.0,
                                 GRB_CONTINUOUS,
                                 fmt::format("I_n{0:d}(idle + c_vel * inter_vel)", robot_nr)));

            m_robot_inter_transition_currents[robot_nr]
                    = std::make_shared<GRBVar>(
                    model.addVar(0.0,
                                 double(m_traits_problem_inputs->robot(robot_nr)->species()->max_possible_current()),
                                 0.0,
                                 GRB_CONTINUOUS,
                                 fmt::format("(I_inter_vel_n{0:d})^p(n)", robot_nr)));



            const std::shared_ptr<const Robot>& robot = m_traits_problem_inputs->robot(robot_nr);
            const Eigen::VectorXf& initial_traits = robot->initial_traits();
            const Eigen::VectorXf& trait_rates_max = robot->species()->trait_rates_max();

            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                if (!robotHasTrait(robot_nr, trait_nr))
                {
                    continue;
                }
                m_robot_trait_consumption_sum_or_max[robot_nr][trait_nr] = std::make_shared<GRBLinExpr>(0.0);
            } // end for trait_nr

            for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
                if (!robotHasAssignedTask(robot_nr, task_nr))
                {
                    continue;
                }

                /******************** Currents ********************/

                m_task_robot_currents_expr[task_nr][robot_nr] = std::make_shared<GRBLinExpr>(
                        m_traits_problem_inputs->robot(robot_nr)->species()->idle_current());

                m_task_robot_currents_expr_var[task_nr][robot_nr] = std::make_shared<GRBVar>(
                        model.addVar(0.0,
                                     double(m_traits_problem_inputs->robot(robot_nr)->species()->max_possible_current()),
                                     0.0,
                                     GRB_CONTINUOUS,
                                     fmt::format("I_k{:d}n{:d}", task_nr, robot_nr)));

                m_task_robot_currents_var[task_nr][robot_nr] = std::make_shared<GRBVar>(
                        model.addVar(0.0,
                                     double(m_traits_problem_inputs->robot(robot_nr)->species()->max_possible_current()),
                                     0.0,
                                     GRB_CONTINUOUS,
                                     fmt::format("I^p_k{:d}n{:d}", task_nr, robot_nr)));

                for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                    if (!robotHasTrait(robot_nr, trait_nr))
                    {
                        continue;
                    }

                    /******************** TRAITS ********************/
                    m_trait_variables[task_nr][robot_nr][trait_nr]
                        = std::make_shared<GRBVar>(model.addVar(0.0,
                                       double(initial_traits(trait_nr)),
                                       0.0,
                                       GRB_CONTINUOUS,
                                       fmt::format("Q_k{:d}n{:d}u{:d}",
                                                   task_nr,
                                                   robot_nr,
                                                   trait_nr)));

                    m_task_robot_trait_durations[task_nr][robot_nr][trait_nr]
                        = std::make_shared<GRBVar>(
                                // Note: static task duration is zero (b/c not involving any rates)
                                // model.addVar(1e-3f,
                                model.addVar(0.0,
                                             robotTraitProvisionable(robot_nr, trait_nr) ? GRB_INFINITY : 0.0,
                                             0.0,
                                             GRB_CONTINUOUS,
                                             fmt::format("t_k{:d}n{:d}u{:d}",
                                             task_nr, robot_nr, trait_nr))
                    );


                    if (robotTraitProvisionable(robot_nr, trait_nr)) {
                        // dynamic trait
                        m_trait_rate_variables[task_nr][robot_nr][trait_nr]
                                = std::make_shared<GRBVar>(model.addVar(0.0,
                                                                        double(trait_rates_max(trait_nr)),
                                                                        0.0,
                                                                        GRB_CONTINUOUS,
                                                                        fmt::format("Qd_k{:d}n{:d}u{:d}",
                                                                                    task_nr,
                                                                                    robot_nr,
                                                                                    trait_nr))
                        );

                    }  // end if

                    /******************** Currents ********************/
                    if (robotHasPositiveTraitCoeff(robot_nr, trait_nr)) {
                        m_task_robot_trait_currents[task_nr][robot_nr][trait_nr] =
                                std::make_shared<GRBVar>(model.addVar(0.0,
                                                                      robotHasAssignedTask(robot_nr, task_nr) ? GRB_INFINITY : 0.0,
                                                                      0.0,
                                                                      GRB_CONTINUOUS,
                                                                      fmt::format("I_Q_k{:d}n{:d}u{:d}",
                                                                                  task_nr, robot_nr, trait_nr)));
                    }

                    if (robotHasPositiveTraitRateCoeff(robot_nr, trait_nr)) {
                        m_task_robot_trait_rate_currents[task_nr][robot_nr][trait_nr] =
                                std::make_shared<GRBVar>(model.addVar(0.0,
                                                                      robotHasAssignedTask(robot_nr, task_nr)
                                                                      ? GRB_INFINITY : 0.0,
                                                                      0.0,
                                                                      GRB_CONTINUOUS,
                                                                      fmt::format("I_Qd_k{:d}n{:d}u{:d}",
                                                                                  task_nr, robot_nr, trait_nr)));
                    }
                }  // end for trait_nr
            }  // end for task_nr
        } // end for robot_nr
        // ----- Robot Related: END ----- //

        return nullptr;
    } // end createVariable()


    std::shared_ptr<const FailureReason> TraitDistributor::createConstraints(GRBModel& model)
    {

        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr)
        {
            if (!taskHasAssignedRobot(task_nr)) {
                continue;
            }

            // note: sum_{k \in K} t_k
            *m_total_task_durations += *m_task_durations[task_nr];

            for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr)
            {
                if (!robotHasAssignedTask(robot_nr, task_nr)) {
                    continue;
                }

                for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr)
                {
                    // if robot doesn't have this trait OR task doesn't require this trait
                    if (!robotHasTrait(robot_nr, trait_nr) || !taskTraitRequired(task_nr, trait_nr))
                    {
                        continue;
                    }

                    if (taskTraitaggregatable(task_nr, trait_nr))
                    {
                        // note: Y_ku = sum_{n \in A_\tau(k)} Q_ku^(n)
                        *m_coalition_traits_sum_expr[task_nr][trait_nr] +=
                                *m_trait_variables[task_nr][robot_nr][trait_nr];
                        // above -- side note: moving make_shared<> to shared_ptr in the constructor caused unexpected behaviors

                    } else {
                        // note: Q_ku^(n) >= Y*_ku
                        model.addConstr(*m_trait_variables[task_nr][robot_nr][trait_nr] >=  // note: >=
                                        (m_traits_problem_inputs->task(task_nr)->desiredTraits())(trait_nr),
                                        fmt::format("Q_k{0:d}n{1:d}u{2:d}>=Ys_k{0:d}n{1:d}u{2:d}",
                                                    task_nr, robot_nr, trait_nr));

                        if (robotTraitProvisionable(robot_nr, trait_nr))
                        {
                            // trait rate must meet MINIMUM requirement
                            // note: Qd_ku^(n) >= Y*d_ku  -- greater than because induce shorter t_ku
                            model.addConstr(*m_trait_rate_variables[task_nr][robot_nr][trait_nr] >=
                                            (m_traits_problem_inputs->task(task_nr)->minimum_trait_rates())(trait_nr),
                                            fmt::format("Qd_k{0:d}n{1:d}u{2:d}>=Ysd_k{0:d}n{1:d}u{2:d}",
                                                        task_nr, robot_nr, trait_nr));
                        }

                    }

                    /*!
                     * For Task Trait Durations of Coalitions
                     */
                    if (robotTraitProvisionable(robot_nr, trait_nr)) {
                            // note: Q_ku^(n) = Qd_ku^(n) * t_ku^(n)
                            // this is to find t_ku^(n)
                            model.addQConstr(*m_trait_variables[task_nr][robot_nr][trait_nr] ==
                            *m_task_robot_trait_durations[task_nr][robot_nr][trait_nr] *
                            *m_trait_rate_variables[task_nr][robot_nr][trait_nr],
                            fmt::format("t_k{:d}n{:d}u{:d}", task_nr, robot_nr, trait_nr));

                            // note: t_ku >= t_ku^(n) forall n
                            // That means when trait is not provisionable, task trait durations = 0.0
                            model.addConstr( *m_task_trait_durations[task_nr][trait_nr] >=
                            *m_task_robot_trait_durations[task_nr][robot_nr][trait_nr],
                            fmt::format("t_k{0:d}u{2:d}>=t_k{0:d}n{1:d}u{2:d}",
                                        task_nr, robot_nr, trait_nr));
                    } else {
                        m_task_robot_trait_durations[task_nr][robot_nr][trait_nr]->set(GRB_DoubleAttr_LB, 0.0);
                        m_task_robot_trait_durations[task_nr][robot_nr][trait_nr]->set(GRB_DoubleAttr_UB, 0.0);
                    }

                    // This is from robot's perspective
                    if (robotTraitExhaustible(robot_nr, trait_nr))
                    {
                        //  robot self trait sum
                        *m_robot_trait_consumption_sum_or_max[robot_nr][trait_nr] += *m_trait_variables[task_nr][robot_nr][trait_nr];
                    }
                }
            }
        }


        for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr) {
                if (robotIsAssigned(robot_nr) && robotHasTrait(robot_nr, trait_nr))
                {
                    model.addConstr(*m_robot_trait_consumption_sum_or_max[robot_nr][trait_nr] <=
                                    m_traits_problem_inputs->robot(robot_nr)->initial_traits()(trait_nr),
                                    fmt::format("init_Q_n{0:d}u{1:d}", robot_nr, trait_nr));
                }
            }
        }

        /*!
         * For Trait Mismatch
         */
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr)
        {
            // if no task has no robot assigned
            if (!taskHasAssignedRobot(task_nr))
            {
                for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr)
                {
                    *m_total_traits_unfulfilled +=
                            m_traits_problem_inputs->task(task_nr)->desiredTraits()(trait_nr);
                    *m_total_trait_rates_unfulfilled +=
                            m_traits_problem_inputs->task(task_nr)->minimum_trait_rates()(trait_nr);
                }

                continue;
            }

            for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr)
            {
                if (!taskTraitRequired(task_nr, trait_nr))
                {
                    continue;
                }

                if (taskTraitRateRequired(task_nr, trait_nr))
                {
                    // note: t_k >= t_q_ku forall u
                    model.addConstr(*m_task_dynamic_durations[task_nr] >= *m_task_trait_durations[task_nr][trait_nr],
                                    fmt::format("t_q_k{0:d}>=t_k{0:d}u{1:d}", task_nr, trait_nr));
                }

                if (taskTraitAllowsCoalition(task_nr, trait_nr))
                {
                    // note: E_ku = Ys_ku - Y_ku
                    model.addConstr(*m_task_traits_unfulfilled[task_nr][trait_nr] ==
                            m_traits_problem_inputs->task(task_nr)->desiredTraits()(trait_nr) -
                            *m_coalition_traits_sum_expr[task_nr][trait_nr],
                            fmt::format("E_k{0:d}u{1:d}", task_nr, trait_nr));

                    // We know cumulative traits can be summed and [0, inf), thus no negative values
                    *m_total_traits_unfulfilled += *m_task_traits_unfulfilled[task_nr][trait_nr];

                    // if dynamic trait
                    if (taskTraitProvisionable(task_nr, trait_nr)) {
                        // note: Y_ku = Yd_ku * t_ku (CRUCIAL: addQConstraints)
                        model.addQConstr(*m_coalition_traits_sum_expr[task_nr][trait_nr] ==
                                         *m_coalition_trait_rates_avg_var[task_nr][trait_nr] *
                                         *m_task_trait_durations[task_nr][trait_nr],
                                         fmt::format("Avg_Yd_constr_k{0:d}u{1:d}", task_nr, trait_nr));

                        // trait rate is only considered when trait is gradually provisioning (i.e., provisionable)
                        // note: non-cumulative is already satisfied (Qdot_ku >= Ydot^*_ku)
                        model.addConstr(*m_task_trait_rates_unfulfilled[task_nr][trait_nr] >=
                                        m_traits_problem_inputs->task(task_nr)->minimum_trait_rates()(trait_nr) -
                                        *m_coalition_trait_rates_avg_var[task_nr][trait_nr],
                                        fmt::format("Edot_k{0:d}u{1:d}>=max(0,Ydot_k{0:d}u{1:d})",
                                                    task_nr, trait_nr));


                        *m_total_trait_rates_unfulfilled += *m_task_trait_rates_unfulfilled[task_nr][trait_nr];
                    }
                }

                // NOTE: if not coalitionable, then the slowest robots rate will be the task duration
                // this is also achievable by (Qdot_ku >= Ydot^*_ku) and t_ku = t_ku^(n) forall n
                // task_trait_rate[task_nr][trait_nr] >= 0, so it will automatically be zero
                // when we specificy minimizing this in the objective.
            }
        }

        /*!
         * For Inter Task Transition Duration Calculation
         */
        // this doesn't take too much of computation time -- relatively.
        std::vector<float> m_robot_inter_distance_ub(numRobots, 0.0);
        // Get Upperbound of Distance
        for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr) {
            if (!robotIsAssigned(robot_nr)) {
                continue;
            }

            for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
                if (robotHasAssignedTask(robot_nr, task_nr)) {
                    const std::shared_ptr<const Robot> &_robot = m_traits_problem_inputs->robot(robot_nr);
                    const std::shared_ptr<const Task> &_task = m_traits_problem_inputs->task(task_nr);

                    // robot init config -> task init config
                    float dist_outbound = _robot->pathLengthQuery(_robot->initialConfiguration(),
                                                                  _task->initialConfiguration());
                    // making sure the duration is out of bound (0 < t < inf)
                    dist_outbound = (dist_outbound < 0.0) ? -std::numeric_limits<float>::infinity() :
                                        dist_outbound;

                    // task term config -> robot init config
                    float dist_inbound = _robot->pathLengthQuery(_task->initialConfiguration(),
                                                                 _robot->initialConfiguration());
                    dist_inbound = (dist_inbound < 0.0) ? -std::numeric_limits<float>::infinity() :
                                    dist_inbound;

                    m_robot_inter_distance_ub[robot_nr] += dist_outbound + dist_inbound;
                }
            }

            model.addQConstr(
                    *m_robot_inter_transition_ub_durations[robot_nr] *
                        *m_robot_inter_transition_velocities[robot_nr] >= m_robot_inter_distance_ub[robot_nr],
                    fmt::format("phi_interUB_n{:d}", robot_nr));
        }

        /*!
         * For Energy Consumption
         * To Consider: 1. Transition + Trait Consumption (2.1, Amount & 2.2, Rate)
         */
        for (unsigned int task_nr = 0; task_nr < numTasks; ++task_nr) {
             if (!taskHasAssignedRobot(task_nr))
             {
                 continue;
             }

             std::shared_ptr<const Robot> widest_robot = nullptr;
             for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr)
             {
                 if (!robotHasAssignedTask(robot_nr, task_nr))
                 {
                     continue;
                 }

                 if (widest_robot == nullptr ||
                        m_traits_problem_inputs->robot(robot_nr)->boundingRadius() > widest_robot->boundingRadius()) {
                    widest_robot = m_traits_problem_inputs->robot(robot_nr);
                 }
                 model.addConstr(
                         *m_task_coalition_velocities[task_nr]
                         <= m_traits_problem_inputs->robot(robot_nr)->species()->max_speed(),
                         fmt::format("v_k{0:d}<=v_n{1:d}", task_nr, robot_nr));

                 // robot current = idle current (added when initialized) + intra-transition current
                 *m_task_robot_currents_expr[task_nr][robot_nr] +=
                         robotSpeedCoeff(robot_nr) * *m_task_coalition_velocities[task_nr];

                 for (unsigned int trait_nr = 0; trait_nr < numTraits; ++trait_nr)
                 {
                     if (!taskTraitRequired(task_nr, trait_nr) || !robotHasTrait(robot_nr, trait_nr))
                     {
                         continue;
                     }


                     if (m_trait_variables[task_nr][robot_nr][trait_nr] != nullptr) {
                         *m_task_robot_currents_expr[task_nr][robot_nr] +=
                                 robotTraitCoeff(robot_nr, trait_nr) * *m_trait_variables[task_nr][robot_nr][trait_nr];
                     }

                     if (m_trait_rate_variables[task_nr][robot_nr][trait_nr] != nullptr) {
                         *m_task_robot_currents_expr[task_nr][robot_nr] +=
                                 robotTraitRateCoeff(robot_nr, trait_nr) * *m_trait_rate_variables[task_nr][robot_nr][trait_nr];
                     }
                 }

                 model.addConstr(
                         *m_task_robot_currents_expr_var[task_nr][robot_nr] ==
                         *m_task_robot_currents_expr[task_nr][robot_nr],
                         fmt::format("I_k{0:d}n{1:d}", task_nr, robot_nr));

                 model.addGenConstrPow(
                        *m_task_robot_currents_expr_var[task_nr][robot_nr],  // xvar
                        *m_task_robot_currents_var[task_nr][robot_nr],  // yvar
                        robotPeukertCoeff(robot_nr),
                        fmt::format("I^p_k{0:d}n{1:d}", task_nr, robot_nr));

                 *m_robot_battery_consumption_expr[robot_nr] +=
                         *m_task_robot_currents_var[task_nr][robot_nr] *
                         *m_task_durations[task_nr];

             }

             for (unsigned int robot_nr = 0; robot_nr < numRobots; ++robot_nr)
             {
                 if (!robotIsAssigned(robot_nr)) {
                     continue;
                 }

                 // Also, idle_current for inter-transition current is too heavy -- takes too much of battery
                 *m_robot_inter_transition_currents_LExpr[robot_nr] =
                         m_traits_problem_inputs->robot(robot_nr)->species()->idle_current() +   // Idle Current
                                 robotSpeedCoeff(robot_nr) * *m_robot_inter_transition_velocities[robot_nr];

                 // Requires a variable to exponentiate LExpr
                 model.addConstr(
                         *m_robot_inter_transition_currents_var[robot_nr] ==
                         *m_robot_inter_transition_currents_LExpr[robot_nr],
                         fmt::format("I_inter_n{1:d}", task_nr, robot_nr));

                // Using Peukert Exponent
                 model.addGenConstrPow(
                         *m_robot_inter_transition_currents_var[robot_nr],  // xvar
                         *m_robot_inter_transition_currents[robot_nr],  // yvar
                         robotPeukertCoeff(robot_nr),
                         fmt::format("I^p_inter_n{1:d}", task_nr, robot_nr));

                 // battery consumption of all inter-transitions for robot_nr
                 *m_robot_battery_consumption_expr[robot_nr] +=
                         *m_robot_inter_transition_currents[robot_nr] *
                         *m_robot_inter_transition_ub_durations[robot_nr];

                 // note: this must be QConstr
                 model.addQConstr(*m_robot_battery_consumption_expr[robot_nr] <=
                                 double(m_traits_problem_inputs->robot(robot_nr)->initial_battery_level()));
             }


             // check length if no path exists
             float path_length = widest_robot->pathLengthQuery(
                     m_traits_problem_inputs->task(task_nr)->initialConfiguration(),
                     m_traits_problem_inputs->task(task_nr)->terminalConfiguration());

             model.addQConstr(*m_task_intra_transition_durations[task_nr] *
                                *m_task_coalition_velocities[task_nr] >= path_length,
                              fmt::format("t_intra_k{:d}", task_nr));

             model.addConstr(*m_task_durations[task_nr] >=
                                *m_task_dynamic_durations[task_nr]
                                + m_traits_problem_inputs->task(task_nr)->staticDuration()
                                + *m_task_intra_transition_durations[task_nr],
                                fmt::format("t_k{0:d}>=t^s_k{0:d}+t^q_k{0:d}+phi_intra_k{0:d}", task_nr));
        } // end for task_nr


        return nullptr;
    }

} // namespace traits