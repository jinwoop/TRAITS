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
#include <mutex>
// External
#include <gurobi_c++.h>
// Local
#include "traits/common/utilities/noncopyable.hpp"
#include "traits/common/utilities/update_model_result.hpp"
// endregion

namespace traits
{
    // region Forward Declarations
    class ParametersBase;
    class FailureReason;
    class NlpSolverResult;
    // endregion

    //! \brief Abstract base class for algorithms that use a NLP formulation
    class NlpSolverBase : public Noncopyable
    {
    public:
        // region Special Member Functions
        NlpSolverBase()                                 = delete;
        NlpSolverBase(NlpSolverBase&&) noexcept         = default;
        virtual ~NlpSolverBase();
        NlpSolverBase& operator=(const NlpSolverBase&)  = delete;
        NlpSolverBase& operator=(NlpSolverBase&&)       = delete;
        // endregion

        //! \returns The number of times the NLP optimization was run
        [[nodiscard]] inline unsigned int numIterations() const;

        //! \brief Solves a NLP problem
        [[nodiscard]] std::shared_ptr<NlpSolverResult> solveNlp(const std::shared_ptr<const ParametersBase>& parameters);

        /*!
         * \brief Resolves the NLP assuming that something has been updated
         */
        [[nodiscard]] std::shared_ptr<NlpSolverResult> resolve(bool reset = false);

        /*!
         * \brief Resolves the NLP assuming that something has been updated
         */
        [[nodiscard]] std::shared_ptr<NlpSolverResult> resolve(const std::shared_ptr<NlpSolverResult>& result,
                                                                bool reset = false);

        /*!
         * Builds a NLP model
         *
         * \param parameters A set of parameters for the NLP model
         *
         * \returns The model
         */
        [[nodiscard]] virtual std::shared_ptr<NlpSolverResult> createModel(const std::shared_ptr<const ParametersBase>& parameters);

        [[nodiscard]] inline std::shared_ptr<GRBModel> model();

        //! Clears and removes all environments
        static void clearEnvironments();

        static void checkEnvironmentErrors();

    protected:
        /*!
         * Sets up the data to be used to generate the model
         *
         * \returns Whether it was successful
         */
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> setupData() = 0;

        //! Set parameters for the NLP \p model
        virtual void setParameters(GRBModel& model, const std::shared_ptr<const ParametersBase>& parameters);

        //! Add variables to the NLP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createVariables(GRBModel& model) = 0;

        //! Add an objective to the NLP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createObjective(GRBModel& model) = 0;

        //! Add constraints to the NLP \p model
        [[nodiscard]] virtual std::shared_ptr<const FailureReason> createConstraints(GRBModel& model) = 0;

        /*!
         * Updates model after each run (Default: does nothing)
         *
         * \param model
         *
         * \returns The result of attempting to update
         */
        [[nodiscard]] virtual UpdateModelResult updateModel(GRBModel& model);

        //! Constructor
        explicit NlpSolverBase(unsigned int count);


        [[nodiscard]] GRBEnv& getEnvironment();

        bool m_return_feasible_on_timeout{};
        std::shared_ptr<GRBModel> m_model;
        unsigned int m_num_iterations{};
        int m_environment_index{};

        static std::mutex s_environment_lock;
        static std::vector<GRBEnv> s_environment_pool;
        static std::vector<bool> s_environment_taken;
    };

    // Inline Functions
    unsigned int NlpSolverBase::numIterations() const
    {
        return m_num_iterations;
    }

    std::shared_ptr<GRBModel> NlpSolverBase::model()
    {
        return m_model;
    }

}  // namespace traits
