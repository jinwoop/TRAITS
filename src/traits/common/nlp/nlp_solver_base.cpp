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
#include "traits/common/nlp/nlp_solver_base.hpp"

// Local
#include "traits/common/nlp/nlp_infeasible.hpp"
#include "traits/common/nlp/nlp_solver_result.hpp"
#include "traits/common/nlp/nlp_timeout.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/failure_reason.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/parameters/parameters_base.hpp"

namespace traits
{
    class DeadlineBase;

    std::mutex NlpSolverBase::s_environment_lock          = std::mutex();
    std::vector<GRBEnv> NlpSolverBase::s_environment_pool = std::vector<GRBEnv>();
    std::vector<bool> NlpSolverBase::s_environment_taken  = std::vector<bool>();

    NlpSolverBase::NlpSolverBase(unsigned int count)
            : m_return_feasible_on_timeout(false)
            , m_num_iterations(count)
            , m_environment_index(-1)
    {}

    NlpSolverBase::~NlpSolverBase()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(m_environment_index >= 0)
        {
            s_environment_taken[m_environment_index] = false;
        }
    }

    std::shared_ptr<NlpSolverResult> NlpSolverBase::solveNlp(const std::shared_ptr<const ParametersBase>& parameters)
    {
        std::shared_ptr<NlpSolverResult> result = createModel(parameters);
        if(result->failure())
        {
            return result;
        }

        while(true)
        {
            result->incrementNumIterations();
            result = resolve(result);
            if(result->failure())
            {
                return result;
            }

            UpdateModelResult update_model_result = updateModel(*m_model);
            switch(update_model_result.type())
            {
                case UpdateModelResultType::e_no_update:
                {
                    return result;
                }
                case UpdateModelResultType::e_failure:
                {
                    return std::make_shared<NlpSolverResult>(update_model_result.failureReason(),
                                                              result->numIterations());
                }
                    // Loop
                case UpdateModelResultType::e_updated:
                {
                    break;
                }
            }
        }
    }

    std::shared_ptr<NlpSolverResult> NlpSolverBase::resolve(bool reset)
    {
        return resolve(std::make_shared<NlpSolverResult>(m_model), reset);
    }

    std::shared_ptr<NlpSolverResult> NlpSolverBase::resolve(const std::shared_ptr<NlpSolverResult>& result,
                                                              bool reset)
    {
        if(reset)
        {
            m_model->reset();
        }
        m_model->update();
        m_model->optimize();

        switch(m_model->get(GRB_IntAttr_Status))
        {
            case GRB_OPTIMAL:
            {
                break;
            }
            case GRB_SUBOPTIMAL:
            {
                Logger::warn("A suboptimal solution was found for the optimization");
                break;
            }
            case GRB_INFEASIBLE:
            {
                Logger::warn("NLP Optimization model determined to be infeasible");
                return std::make_shared<NlpSolverResult>(std::make_shared<NlpInfeasible>(), result->numIterations());
            }
            case GRB_TIME_LIMIT:
            {
                if(m_return_feasible_on_timeout)
                {
                    if(m_model->get(GRB_IntAttr_SolCount) > 0)
                    {
                        Logger::warn("NLP Optimization timed out. Feasible solutions available.");
                        m_model->set(GRB_IntParam_SolutionNumber, 0);
                        break;
                    }
                    else
                    {
                        Logger::warn("NLP Optimization timed out. No feasible solution.");
                        return std::make_shared<NlpSolverResult>(std::make_shared<NlpTimeout>(),
                                                                  result->numIterations());
                    }
                }
                else
                {
                    Logger::warn("NLP Optimization timed out");
                    return std::make_shared<NlpSolverResult>(std::make_shared<NlpTimeout>(), result->numIterations());
                }
            }
            case GRB_UNBOUNDED:
            {
                Logger::warn("NLP Optimization model determined to be unbounded");
                return std::make_shared<NlpSolverResult>(std::make_shared<NlpInfeasible>(), result->numIterations());
            }
            case GRB_INF_OR_UNBD:
            {
                Logger::warn("NLP Optimization model determined to be infeasible or unbounded");
                return std::make_shared<NlpSolverResult>(std::make_shared<NlpInfeasible>(), result->numIterations());
            }
        }
        return result;
    }

    std::shared_ptr<NlpSolverResult> NlpSolverBase::createModel(
            const std::shared_ptr<const ParametersBase>& parameters)
    {
        GRBEnv& env = getEnvironment();
        m_model     = std::make_shared<GRBModel>(env);
        setParameters(*m_model, parameters);

        if(std::shared_ptr<const FailureReason> failure_reason = setupData(); failure_reason)
        {
            return std::make_shared<NlpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createVariables(*m_model); failure_reason)
        {
            return std::make_shared<NlpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createConstraints(*m_model); failure_reason)
        {
            return std::make_shared<NlpSolverResult>(failure_reason);
        }
        if(std::shared_ptr<const FailureReason> failure_reason = createObjective(*m_model); failure_reason)
        {
            return std::make_shared<NlpSolverResult>(failure_reason);
        }

        m_model->update();
        return std::make_shared<NlpSolverResult>(m_model);
    }

    UpdateModelResult NlpSolverBase::updateModel(GRBModel& model)
    {
        return UpdateModelResult(UpdateModelResultType::e_no_update);
    }

    void NlpSolverBase::setParameters(GRBModel& model, const std::shared_ptr<const ParametersBase>& parameters)
    {
        if(parameters->contains(constants::k_return_feasible_on_timeout) &&
           parameters->get<bool>(constants::k_return_feasible_on_timeout))
        {
            m_return_feasible_on_timeout = true;
            model.set(GRB_IntParam_PoolSolutions, 1);  // Only need one
        }

        if(parameters->contains(constants::k_nlp_timeout) && parameters->get<float>(constants::k_nlp_timeout) > 0.0f)
        {
            model.set(GRB_DoubleParam_TimeLimit,
                      static_cast<double>(parameters->get<float>(constants::k_nlp_timeout)));
        }
    }

    GRBEnv& NlpSolverBase::getEnvironment()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(s_environment_pool.empty())
        {
            GRBEnv& env = s_environment_pool.emplace_back(true);
            env.set(GRB_IntParam_LogToConsole, 0);
            env.start();
        }
        return s_environment_pool.front();
    }

    void NlpSolverBase::clearEnvironments()
    {
        std::lock_guard<std::mutex> lock(s_environment_lock);
        if(std::any_of(s_environment_taken.begin(),
                       s_environment_taken.end(),
                       [](bool b) -> bool  // Hopefully the compiler is smart about this...
                       {
                           return b;
                       }))
        {
            throw createLogicError("Attempting to clear gurobi environments while some are still in use");
        }
        s_environment_pool.clear();
        s_environment_taken.clear();
    }

    void NlpSolverBase::checkEnvironmentErrors()
    {
        for(GRBEnv& env: s_environment_pool)
        {
            Logger::warn(env.getErrorMsg());
        }
    }

}  // namespace traits
