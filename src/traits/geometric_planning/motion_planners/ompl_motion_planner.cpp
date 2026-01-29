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
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"

// External
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
// Local
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/common/utilities/json_tree_factory.hpp"
#include "traits/common/utilities/timer_runner.hpp"
#include "traits/geometric_planning/configurations/configuration_base.hpp"
#include "traits/geometric_planning/configurations/ompl_configuration.hpp"
#include "traits/geometric_planning/environments/ompl_environment.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"
#include "traits/geometric_planning/query_results/motion_planner_query_result_base.hpp"
#include "traits/parameters/parameters_base.hpp"
#include "traits/parameters/parameters_factory.hpp"

namespace traits
{
    unsigned int TraitsOmplMotionPlanner::s_num_failures = 0;

    TraitsOmplMotionPlanner::TraitsOmplMotionPlanner(OmplMotionPlannerType ompl_motion_planner_type,
                                         const std::shared_ptr<const ParametersBase>& parameters,
                                         const std::shared_ptr<TraitsOmplEnvironment>& environment)
            : m_parameters(parameters)
            , m_environment(environment)
            , m_simple_setup(nullptr)
            , m_ompl_motion_planner_type(ompl_motion_planner_type)
    {
        ompl::msg::noOutputHandler();

        // Simple Setup
        auto ompl_environment = std::dynamic_pointer_cast<TraitsOmplEnvironment>(m_environment);
        m_simple_setup        = std::make_unique<ompl::geometric::SimpleSetup>(ompl_environment->stateSpace());
        m_simple_setup->setStateValidityChecker(ompl_environment);
        std::shared_ptr<ompl::base::Planner> motion_planner;
        switch(ompl_motion_planner_type)
        {
            case OmplMotionPlannerType::e_prm:
            {
                motion_planner = std::make_shared<ompl::geometric::PRM>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_prm_star:
            {
                motion_planner = std::make_shared<ompl::geometric::PRMstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_prm:
            {
                motion_planner = std::make_shared<ompl::geometric::LazyPRM>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_prm_star:
            {
                motion_planner = std::make_shared<ompl::geometric::LazyPRMstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt:
            {
                motion_planner = std::make_shared<ompl::geometric::RRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt_star:
            {
                motion_planner = std::make_shared<ompl::geometric::RRTstar>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_parallel_rrt:
            {
                motion_planner = std::make_shared<ompl::geometric::pRRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_rrt_connect:
            {
                motion_planner = std::make_shared<ompl::geometric::RRTConnect>(m_simple_setup->getSpaceInformation());
                break;
            }
            case OmplMotionPlannerType::e_lazy_rrt:
            {
                motion_planner = std::make_shared<ompl::geometric::LazyRRT>(m_simple_setup->getSpaceInformation());
                break;
            }
            default:
            {
                throw createLogicError("Unknown motion planner type");
            }
        }
        m_simple_setup->setPlanner(motion_planner);
    }

    void TraitsOmplMotionPlanner::init()
    {
        static bool first = true;
        if(first)
        {
            first                                 = false;
            ParametersFactory& parameters_factory = ParametersFactory::instance();
            TraitsOmplEnvironment::init();

            // region OmplMotionPlanner
            JsonTreeFactory<TraitsOmplMotionPlanner>::instance().set(
                    constants::k_ompl_motion_planner,
                    [&parameters_factory](const nlohmann::json& j) -> std::shared_ptr<TraitsOmplMotionPlanner>
                    {
                        traits::json_ext::validateJson(
                                j,
                                {{constants::k_algorithm_parameters, nlohmann::json::value_t::object},
                                 {constants::k_environment_parameters, nlohmann::json::value_t::object}});
                        auto parameters = parameters_factory.create(ParametersFactory::Type::e_motion_planner,
                                                                    j.at(constants::k_algorithm_parameters));

                        auto environment = JsonTreeFactory<TraitsOmplEnvironment>::instance().create(
                                j.at(traits::constants::k_environment_parameters));
                        return std::make_shared<TraitsOmplMotionPlanner>(
                                parameters->get<traits::OmplMotionPlannerType>(constants::k_ompl_mp_algorithm),
                                parameters,
                                environment);
                    });
            // endregion
        }
    }

    const std::shared_ptr<ompl::base::SpaceInformation>& TraitsOmplMotionPlanner::spaceInformation() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_simple_setup->getSpaceInformation();
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> TraitsOmplMotionPlanner::computeMotionPlan(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        if(!m_simple_setup)
        {
            throw createLogicError("Motion planning not initialized");
        }

        const auto initial_configuration_ompl =
                std::dynamic_pointer_cast<const OmplConfiguration>(initial_configuration);
        const auto goal_configuration_ompl = std::dynamic_pointer_cast<const OmplConfiguration>(goal_configuration);

        // Clears internal from previous query
        m_simple_setup->getPlanner()->clearQuery();
        m_simple_setup->getProblemDefinition()->clearSolutionPaths();

        // Set start and goal
        ompl::base::ScopedStatePtr scoped_initial_state =
                initial_configuration_ompl->convertToScopedStatePtr(m_simple_setup->getStateSpace());
        if(!scoped_initial_state->satisfiesBounds())
        {
            throw createLogicError("Initial state doesn't respect the bounds of the state space");
        }
        m_simple_setup->setStartState(*scoped_initial_state);
        m_simple_setup->setGoal(goal_configuration_ompl->convertToGoalPtr(m_simple_setup->getSpaceInformation()));

        // Set the radius of the robot
        m_environment->lock();
        m_environment->setSpecies(species);
        const ompl::base::PlannerStatus status = m_simple_setup->solve(ompl::base::plannerOrTerminationCondition(
                ompl::base::timedPlannerTerminationCondition(m_parameters->get<float>(constants::k_timeout)),
                ompl::base::CostConvergenceTerminationCondition(
                        m_simple_setup->getProblemDefinition(),
                        m_parameters->get<unsigned int>(constants::k_solutions_window),
                        m_parameters->get<float>(constants::k_convergence_epsilon))));

        switch(status.operator ompl::base::PlannerStatus::StatusType())
        {
            case ompl::base::PlannerStatus::INVALID_START:
            {
                throw createLogicError("Invalid initial configuration provided to the motion planner");
            }
            case ompl::base::PlannerStatus::INVALID_GOAL:
            {
                throw createLogicError("Invalid goal configuration provided to the motion planner");
            }
            case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
            {
                throw createLogicError("Unrecognized goal type provided to the motion planner");
            }
            case ompl::base::PlannerStatus::TIMEOUT:
            {
                // Clear the species from the environment
                m_environment->setSpecies(nullptr);
                m_environment->unlock();
                ++s_num_failures;
                Logger::warn("Motion planner timed out");
                return std::make_shared<const OmplMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_timeout,
                                                                            nullptr);
            }
            case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION:
            {
                // Clear the species from the environment
                m_environment->setSpecies(nullptr);
                m_environment->unlock();
                ++s_num_failures;
                Logger::warn("Motion planning returned an approximate solution. This is considered a failure as they "
                             "contain jumps.");
                return std::make_shared<const OmplMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_failure,
                                                                            nullptr);
            }
            case ompl::base::PlannerStatus::EXACT_SOLUTION:
            {
                break;
            }
            case ompl::base::PlannerStatus::CRASH:
            {
                throw createLogicError("Motion planner crashed");
            }
            case ompl::base::PlannerStatus::ABORT:
            {
                throw createLogicError("Motion planner aborted");
            }
            default:
            {
                throw createLogicError("Unknown motion planner status");
            }
        }

        if(m_simple_setup->haveSolutionPath())
        {
            if(m_parameters->get<bool>(constants::k_simplify_path))
            {
                m_simple_setup->simplifySolution(m_parameters->get<float>(constants::k_simplify_path_timeout));
            }
            // Clear the species from the environment
            m_environment->setSpecies(nullptr);
            m_environment->unlock();
            const ompl::geometric::PathGeometric& path = m_simple_setup->getSolutionPath();
            auto path_ptr                              = std::make_shared<const ompl::geometric::PathGeometric>(path);
            return std::make_shared<const OmplMotionPlannerQueryResult>(MotionPlannerQueryStatus::e_success, path_ptr);
        }
        else
        {
            // Should not be able to get here
            throw createLogicError("How?");
        }
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> TraitsOmplMotionPlanner::query(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        std::lock_guard lock(m_mutex);
        TimerRunner timer_runner(constants::k_motion_planning_time);
        if(std::shared_ptr<const MotionPlannerQueryResultBase> result =
                    getMemoized(species, initial_configuration, goal_configuration);
                result != nullptr)
        {
            return result;
        }

        // Compute and memoize
        std::shared_ptr<const MotionPlannerQueryResultBase> result =
                computeMotionPlan(species, initial_configuration, goal_configuration);
        auto iter = m_memoization.emplace(std::weak_ptr<const Species>(species),
                                          std::make_tuple(initial_configuration, goal_configuration, result));
        return result;
    }

    float TraitsOmplMotionPlanner::durationQuery(const std::shared_ptr<const Species>& species,
                                           const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                           const std::shared_ptr<const ConfigurationBase>& goal_configuration)
    {
        if(std::shared_ptr<const MotionPlannerQueryResultBase> result =
                    query(species, initial_configuration, goal_configuration);
                result != nullptr)
        {
            return result->duration(species->max_speed());
        }
        return -1.0f;
    }

    bool TraitsOmplMotionPlanner::isMemoized(const std::shared_ptr<const Species>& species,
                                       const std::shared_ptr<const ConfigurationBase>& initial_configuration,
                                       const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        std::lock_guard lock(m_mutex);
        TimerRunner timer_runner(constants::k_motion_planning_time);
        return getMemoized(species, initial_configuration, goal_configuration) != nullptr;
    }

    std::shared_ptr<const MotionPlannerQueryResultBase> TraitsOmplMotionPlanner::getMemoized(
            const std::shared_ptr<const Species>& species,
            const std::shared_ptr<const ConfigurationBase>& initial_configuration,
            const std::shared_ptr<const ConfigurationBase>& goal_configuration) const
    {
        if(!m_memoization.contains(species))
        {
            return nullptr;
        }

        auto range = m_memoization.equal_range(species);
        for(auto iter = range.first; iter != range.second; ++iter)
        {
            const MemoizationValue& mv = iter->second;
            if(*std::get<0>(mv) != *initial_configuration)
            {
                continue;
            }
            if(*std::get<1>(mv) != *goal_configuration)
            {
                continue;
            }
            return std::get<2>(mv);
        }

        return nullptr;
    }

    void TraitsOmplMotionPlanner::clearCache()
    {
        m_memoization.clear();
    }

    unsigned int TraitsOmplMotionPlanner::numFailures()
    {
        return s_num_failures;
    }
}  // namespace traits


