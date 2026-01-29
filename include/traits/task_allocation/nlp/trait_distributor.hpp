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
#pragma once

// region Includes
// Global
#include <memory>
#include <vector>
// External
#include <nlohmann/json.hpp>
#include <Eigen/Core>
// Local
#include "traits/problem_inputs/traits_problem_inputs.hpp"

#include "traits/common/nlp/nlp_solver_base.hpp"
#include "traits/common/nlp/nlp_solver_result.hpp"

#include "traits/task.hpp"
#include "traits/robot.hpp"
#include "traits/species.hpp"

#include <traits/common/utilities/logger.hpp>
// endregion

namespace traits
{
    // region Forward Declarations
    class TraitsProblemInputs;
    class Robot;
    class Species;
    class TraitDistributorResult;
    class TaskTraitAllocation;
    // endregion


    // region for trait variable info class
    class TraitVariableInfo{
    public:
        TraitVariableInfo(unsigned int robot_nr,
                          unsigned int task_nr,
                          unsigned int trait_nr,
                          GRBVar trait_var,
                          GRBVar trait_rate_var)
                      : m_robot_nr(robot_nr)
                      , m_task_nr(task_nr)
                      , m_trait_nr(trait_nr)
                      , m_trait_var(trait_var)
                      , m_trait_rate_var(trait_rate_var)
        {}

        [[nodiscard]] unsigned int robot_nr() const
        {
            return m_robot_nr;
        }

        [[nodiscard]] unsigned int task_nr() const
        {
            return m_task_nr;
        }

        [[nodiscard]] unsigned int trait_nr() const
        {
            return m_trait_nr;
        }

        [[nodiscard]] GRBVar& trait_variable() {
            return m_trait_var;
        }

        [[nodiscard]] GRBVar& trait_rate_variable() {
            return m_trait_rate_var;
        }

    protected:
        unsigned int m_robot_nr;
        unsigned int m_task_nr;
        unsigned int m_trait_nr;
        GRBVar m_trait_var;
        GRBVar m_trait_rate_var;

    };

    // endregion

    /*!
     *
     * \class TraitDistributor
     * \brief Container for a deterministic schedule for a set of tasks with constraints
     */
    class TraitDistributor: public NlpSolverBase {
      public:
        // region Special Member Functions
        //! \brief Default Constructor
        TraitDistributor() = delete;
//        TraitDistributor(const TraitDistributor &) = delete;
        TraitDistributor(TraitDistributor &&) noexcept = default;
        ~TraitDistributor() = default;
//        TraitDistributor &operator=(const TraitDistributor &) = delete;
        TraitDistributor &operator=(TraitDistributor &) noexcept = default;
        // endregion

        /*!
         * \brief Full Constructor
         */
        TraitDistributor(const Eigen::MatrixXf& allocation,
                          const std::shared_ptr<const TraitsProblemInputs>& problem_inputs);


        [[nodiscard]] inline std::vector<std::vector<std::vector<float>>>& robot_task_traits();
        [[nodiscard]] inline std::vector<std::vector<std::vector<float>>>& robot_task_trait_rates();
        [[nodiscard]] inline float trait_deficiencies() const;
        [[nodiscard]] inline float trait_rate_deficiencies() const;

        [[nodiscard]] inline bool taskTraitAllowsCoalition(unsigned int task_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool taskTraitaggregatable(unsigned int task_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool taskHasAssignedRobot(unsigned int task_nr);
        [[nodiscard]] inline bool robotTraitProvisionable(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool robotTraitExhaustible(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool robotHasAssignedTask(unsigned int robot_nr, unsigned int task_nr);
        [[nodiscard]] inline bool robotIsAssigned(unsigned int robot_nr);
        [[nodiscard]] inline bool taskTraitProvisionable(unsigned int task_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool taskTraitRequired(unsigned int task_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool taskTraitRateRequired(unsigned int task_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool robotHasTrait(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool robotHasPositiveTraitCoeff(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline bool robotHasPositiveTraitRateCoeff(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline float robotTraitCoeff(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline float robotTraitRateCoeff(unsigned int robot_nr, unsigned int trait_nr);
        [[nodiscard]] inline float robotSpeedCoeff(unsigned int robot_nr);
        [[nodiscard]] inline float robotPeukertCoeff(unsigned int robot_nr);


        [[nodiscard]] std::shared_ptr<const TaskTraitAllocation> createTaskTraitAllocations();
        [[nodiscard]] std::shared_ptr<const TraitDistributorResult> computeTraitDistribution();
        [[nodiscard]] std::shared_ptr<const TraitDistributorResult> solve();


        // Override from NlpSolverBase
        std::shared_ptr<const FailureReason> setupData();
        std::shared_ptr<const FailureReason> createVariables(GRBModel& model);
        std::shared_ptr<const FailureReason> createObjective(GRBModel& model);
        std::shared_ptr<const FailureReason> createConstraints(GRBModel& model) override;

        [[nodiscard]] nlohmann::json serializeToJson(
                const std::shared_ptr<const TraitsSchedulerProblemInputs>& problem_inputs) const;

       protected:
        // array for GRBVars
        const double A_ZERO[1] = {0.0};
        const double A_GRB_INF[1] = {GRB_INFINITY};
        const double A_GRB_INF_N[1] = {-GRB_INFINITY};
        const char A_GRB_CONT[1] = {GRB_CONTINUOUS};

        // TRAITS
        Eigen::MatrixXf m_allocation;
        std::shared_ptr<const TraitsProblemInputs> m_traits_problem_inputs;

        // Nonlinear Program
        std::shared_ptr<const NlpSolverBase> m_task_distribution_solver;
        std::shared_ptr<const NlpSolverResult> m_task_distribution_result;

        // Robots
        std::vector<std::vector<std::vector< std::shared_ptr<GRBVar>>>> m_trait_variables;
        std::vector<std::vector<std::vector<std::shared_ptr<GRBVar>>>> m_trait_rate_variables;
        std::vector<std::vector<std::shared_ptr<GRBLinExpr>>> m_robot_trait_consumption_sum_or_max;


        // Trait and Trait Rates (Coalition)
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_coalition_trait_rates_avg_var;  // needed for task duration
        std::vector<std::vector<std::shared_ptr<GRBLinExpr>>> m_coalition_traits_sum_expr;

        // Traits Unfulfilled
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_traits_unfulfilled;
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_trait_rates_unfulfilled; // note: keep it?
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_traits_unfulfilled_min;
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_trait_rates_unfulfilled_min;

        // Trait Mismatch (Deficiency / Discrepancy)
        std::shared_ptr<GRBLinExpr> m_total_traits_unfulfilled;
        std::shared_ptr<GRBLinExpr> m_total_trait_rates_unfulfilled;  // note: keep it?

        // Task Durations
        std::vector<std::vector<std::vector<std::shared_ptr<GRBVar>>>> m_task_robot_trait_durations;
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_trait_durations;
        std::vector<std::shared_ptr<GRBVar>> m_task_dynamic_durations;
        std::vector<std::shared_ptr<GRBVar>> m_task_durations;
        std::shared_ptr<GRBLinExpr> m_total_task_durations;

       // Speed (Velocity)
        std::vector<std::shared_ptr<GRBVar>> m_task_coalition_velocities;
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_robot_velocity_currents;
        std::vector<std::shared_ptr<GRBVar>> m_robot_inter_transition_velocities;
        std::vector<std::shared_ptr<GRBVar>> m_robot_inter_transition_ub_durations;
        std::vector<std::shared_ptr<GRBVar>> m_task_intra_transition_durations;

        // Energy
        std::vector<std::vector<std::vector<std::shared_ptr<GRBVar>>>> m_task_robot_trait_currents;  // I_Q_knu = f(Q)
        std::vector<std::vector<std::vector<std::shared_ptr<GRBVar>>>> m_task_robot_trait_rate_currents;  // I_Qd_knu = f(Qd)
        std::vector<std::vector<std::shared_ptr<GRBLinExpr>>> m_task_robot_currents_expr;  // I_kn = sum(I_knu) + I_vel_kn
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_robot_currents_expr_var;   // I_kn -- var (intermediate variable for GenConstrPow)
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_robot_currents_var;   // I_kn^p
        std::vector<std::shared_ptr<GRBLinExpr>> m_robot_inter_transition_currents_LExpr;  // I_idle^(n) + c_v^(n) * inter_vel^(n)
        std::vector<std::shared_ptr<GRBVar>> m_robot_inter_transition_currents_var;  // LExpr var (intermediate variable for GenConstrPow)
        std::vector<std::shared_ptr<GRBVar>> m_robot_inter_transition_currents;  // I_inter_vel_n -- var
        std::vector<std::shared_ptr<GRBQuadExpr>> m_robot_battery_consumption_expr;  // C_n


        // NLP Result -- Thins to return
        std::vector<std::vector<std::vector<float>>> m_result_robot_task_traits;
        std::vector<std::vector<std::vector<float>>> m_result_robot_task_trait_rates;

        std::vector<std::vector<float>> m_result_coalition_traits;
        std::vector<std::vector<float>> m_result_coalition_trait_rates;

        std::vector<float> m_result_task_dynamic_durations;
        std::vector<float> m_result_task_total_durations;
        std::vector<float> m_result_task_intra_transitions;
        std::vector<std::vector<float>> m_result_task_trait_durations;
        std::vector<float> m_result_robot_battery_consumptions;
        std::vector<float> m_result_robot_inter_task_transition_velocities;

        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_trait_deficiencies;
        std::vector<std::vector<std::shared_ptr<GRBVar>>> m_task_trait_rate_deficiencies;
        float m_result_trait_deficiencies;
        float m_result_trait_rate_deficiencies;

        float m_gamma;
        float m_Esum;
        float m_Edotsum;

        unsigned int numTasks;
        unsigned int numRobots;
        unsigned int numTraits;

    };

    std::vector<std::vector<std::vector<float>>>& TraitDistributor::robot_task_traits()
    {
       return m_result_robot_task_traits;
    }

    std::vector<std::vector<std::vector<float>>>& TraitDistributor::robot_task_trait_rates()
    {
        return m_result_robot_task_trait_rates;
    }

    float TraitDistributor::trait_deficiencies() const
    {
        return m_result_trait_deficiencies;
    }

    float TraitDistributor::trait_rate_deficiencies() const
    {
        return m_result_trait_rate_deficiencies;
    }

    // Rename: robotProvisionableTrait
    bool TraitDistributor::robotTraitProvisionable(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->traits_provisionable().row(trait_nr)(0) > 0.5f;
    }

    bool TraitDistributor::robotTraitExhaustible(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->traits_exhaustible().row(trait_nr)(0) > 0.5f;
    }

    bool TraitDistributor::taskTraitAllowsCoalition(unsigned int task_nr, unsigned int trait_nr)
    {
        return taskTraitaggregatable(task_nr, trait_nr);
    }

    bool TraitDistributor::taskTraitaggregatable(unsigned int task_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->task(task_nr)->traits_aggregatable()(trait_nr) > 0.5f;
    }

    bool TraitDistributor::taskHasAssignedRobot(unsigned int task_nr)
    {
        return m_allocation.row(task_nr).sum() > 0.5f;
    }

    bool TraitDistributor::robotHasAssignedTask(unsigned int robot_nr, unsigned int task_nr)
    {
        return m_allocation(task_nr, robot_nr) > 0.5f;
    }

    bool TraitDistributor::robotIsAssigned(unsigned int robot_nr)
    {
        return m_allocation.col(robot_nr).sum() > 0.5f;
    }

    bool TraitDistributor::taskTraitProvisionable(unsigned int task_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->task(task_nr)->minimum_trait_rates()(trait_nr) > 0.0f;
    }

    bool TraitDistributor::taskTraitRequired(unsigned int task_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->task(task_nr)->desiredTraits()(trait_nr) > 0.0f;
    }

    bool TraitDistributor::taskTraitRateRequired(unsigned int task_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->task(task_nr)->minimum_trait_rates()(trait_nr) > 0.0f;
    }

    bool TraitDistributor::robotHasTrait(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->initial_traits()(trait_nr) > 0.0f;
    }

    bool TraitDistributor::robotHasPositiveTraitCoeff(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->current_trait_functions_coeff()(trait_nr) > 0.0f;
    }

    float TraitDistributor::robotTraitCoeff(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->current_trait_functions_coeff()(trait_nr);
    }

    bool TraitDistributor::robotHasPositiveTraitRateCoeff(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->current_trait_rate_functions_coeff()(trait_nr) > 0.0f;
    }

    float TraitDistributor::robotTraitRateCoeff(unsigned int robot_nr, unsigned int trait_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->current_trait_rate_functions_coeff()(trait_nr);
    }

    float TraitDistributor::robotSpeedCoeff(unsigned int robot_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->speed_coeff();
    }

    float TraitDistributor::robotPeukertCoeff(unsigned int robot_nr)
    {
        return m_traits_problem_inputs->robot(robot_nr)->species()->peukert_coeff();
    }

}  // namespace traits
