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
#include "traits/problem_inputs/traits_problem_inputs.hpp"

// Global
#include <fstream>
// External
#include <fmt/format.h>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
// Local
#include "traits/common/milp/milp_solver_base.hpp"
#include "traits/common/utilities/constants.hpp"
#include "traits/common/utilities/custom_views.hpp"
#include "traits/common/utilities/error.hpp"
#include "traits/common/utilities/json_extension.hpp"
#include "traits/common/utilities/json_tree_factory.hpp"
#include "traits/common/utilities/logger.hpp"
#include "traits/common/utilities/ranges_extension.hpp"
// #include "traits/common/utilities/std_extension.hpp"
// #include "traits/common/utilities/time_keeper.hpp"
#include "traits/geometric_planning/configurations/configuration_base.hpp"
#include "traits/geometric_planning/configurations/euclidean_graph_configuration.hpp"
#include "traits/geometric_planning/configurations/se2_ompl_configuration.hpp"
#include "traits/geometric_planning/configurations/se3_ompl_configuration.hpp"
#include "traits/geometric_planning/motion_planners/ompl_motion_planner.hpp"
#include "traits/geometric_planning/motion_planning_enums.hpp"
#include "traits/parameters/parameters_factory.hpp"
#include "traits/problem_inputs/scheduler_problem_inputs.hpp"
#include "traits/scheduling/milp/milp_scheduler.hpp"
#include "traits/scheduling/scheduler_result.hpp"
#include "traits/scheduling/milp/schedule.hpp"
#include "traits/task.hpp"
#include "traits/species.hpp"
#include "traits/task_allocation/itags/robot_traits_matrix_reduction.hpp"
#include "traits/task_allocation/task_allocation_math.hpp"
#include "traits/task_planning/sas/sas_action.hpp"
#include "traits/geometric_planning/environments/ompl_environment.hpp"
#include "traits/scheduling/deadline_base.hpp"
#include "traits/scheduling/absolute_deadline.hpp"
#include "traits/scheduling/relative_deadline.hpp"

namespace traits
{
    TraitsProblemInputs::TraitsProblemInputs(const ThisIsProtectedTag&)
            : m_task_configuration_type(ConfigurationType::e_unknown)
            , m_ompl_state_space_type(OmplStateSpaceType::e_unknown)
            , m_graph_type(GraphType::e_unknown)
    {}

    TraitsProblemInputs::~TraitsProblemInputs()
    {
        // Clear the created gurobi environments
        MilpSolverBase::clearEnvironments();
        // Clear the mp cache to remove shared_ptr cycle species->mp->species
        for(std::shared_ptr<TraitsOmplMotionPlanner>& motion_planner: m_motion_planners)
        {
            motion_planner->clearCache();
        }
    }

    void TraitsProblemInputs::checkConfiguration(const std::shared_ptr<const ConfigurationBase>& configuration) const
    {
        switch(m_task_configuration_type)
        {
            case ConfigurationType::e_ompl:
            {
                switch(m_ompl_state_space_type)
                {
                    case OmplStateSpaceType::e_se2:
                    {
                        const auto& se2_configuration =
                                std::dynamic_pointer_cast<const Se2OmplConfiguration>(configuration);
                        if(!se2_configuration)
                        {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    case OmplStateSpaceType::e_se3:
                    {
                        const auto& se3_configuration =
                                std::dynamic_pointer_cast<const Se3OmplConfiguration>(configuration);
                        if(!se3_configuration)
                        {
                            throw createLogicError("Configuration state space type does not match the central one");
                        }
                        break;
                    }
                    default:
                    {
                        throw createLogicError("Unknown ompl state space type");
                    }
                }
                break;
            }
            case ConfigurationType::e_graph:
            {
                switch(m_graph_type)
                {
                    case GraphType::e_euclidean:
                    {
                        auto pgc = std::dynamic_pointer_cast<const EuclideanGraphConfiguration>(configuration);
                        if(!pgc)
                        {
                            throw createLogicError("Configuration graph type does not match the central one");
                        }
                        break;
                    }
                    case GraphType::e_grid:
                    {
                        throw createLogicError("Grid Configurations Not Implemented");
                    }
                    default:
                    {
                        throw createLogicError("Unknown graph type");
                    }
                }
                break;
            }
            default:
            {
                throw createLogicError("Unknown task configuration type");
            }
        }
    }

    void TraitsProblemInputs::loadMotionPlanners(const nlohmann::json& j)
    {
        if(!j.is_array())
        {
            throw createLogicError("'motion_planners' should be an array of objects");
        }

        TraitsOmplMotionPlanner::init();
        m_motion_planners.reserve(j.size());
        for(const nlohmann::json& individual_mp: j)
        {
            m_motion_planners.emplace_back(JsonTreeFactory<TraitsOmplMotionPlanner>::instance().create(individual_mp));

            // Configuration type means OMPL or Graph
            // First MP
            if(m_task_configuration_type == ConfigurationType::e_unknown)
            {
                m_task_configuration_type = m_motion_planners.back()->environment()->configurationType();
            } else if(m_task_configuration_type != m_motion_planners.back()->environment()->configurationType()) {
                // Any that do not agree with previous motion planners on the configuration space
                throw createLogicError("Cannot load environments of different configuration types");
            }

            if(m_task_configuration_type == ConfigurationType::e_ompl)
            {
                const auto& ompl_environment =
                        std::dynamic_pointer_cast<TraitsOmplEnvironment>(m_motion_planners.back()->environment());
                if(m_ompl_state_space_type == OmplStateSpaceType::e_unknown) {
                    m_ompl_state_space_type = ompl_environment->stateSpaceType();
                } else if(m_ompl_state_space_type != ompl_environment->stateSpaceType()) {
                    throw createLogicError("Cannot load OMPL environments with different state space types");
                }
            }
        }
    }

    void TraitsProblemInputs::createTasks(const std::vector<std::shared_ptr<SasAction>>& grounded_sas_actions,
                                         const nlohmann::json& j)
    {
        m_tasks.reserve(grounded_sas_actions.size());
        for(const std::shared_ptr<SasAction>& action: grounded_sas_actions)
        {
            if(!j.contains(action->name())) {
                throw createLogicError(
                        fmt::format("No associated trait or geometric data for task '{0:s}'", action->name()));
            }

            const nlohmann::json& task_associations_j = j.at(action->name());

            const Eigen::VectorXf desired_traits =
                    task_associations_j.at(constants::k_desired_traits).get<Eigen::VectorXf>();
            const nlohmann::json& initial_configuration_j = task_associations_j.at(constants::k_initial_configuration);
            const nlohmann::json& terminal_configuration_j =
                    task_associations_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                    initial_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(initial_configuration);

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                    terminal_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(terminal_configuration);

            m_tasks.push_back(
                    std::make_shared<const Task>(action, desired_traits, initial_configuration, terminal_configuration));
        }
    }

    std::pair<std::map<std::string, std::shared_ptr<const Species>>, unsigned int> TraitsProblemInputs::loadSpecies(
            const nlohmann::json& j)
    {
        if(m_motion_planners.empty())
        {
            Logger::warn("Loading species without loading motion planners first");
        }

        std::map<std::string, std::shared_ptr<const Species>> rv;
        unsigned int num_traits;
        bool num_traits_set = false;

        m_species.reserve(j.size());
        for(const nlohmann::json& species_j: j)
        {
            m_species.push_back(Species::loadJson(species_j, m_motion_planners));
        }
        // Build map to use for loading the robots down below
        for(const std::shared_ptr<const Species>& s: m_species)
        {
            rv[s->name()] = s;
            if(!num_traits_set)
            {
                num_traits     = s->traits_max().size();
                num_traits_set = true;
            }
        }

        m_fastest_species_speed = m_species |
                                   ::ranges::views::transform(
                                           [](const std::shared_ptr<const Species>& species) -> float
                                           {
                                               return species->max_speed();
                                           }) |
                                   ranges_ext::max<float>();

        m_slowest_species_speed = m_species |
                                    ::ranges::views::transform(
                                            [](const std::shared_ptr<const Species>& species) -> float
                                            {
                                                return species->max_speed();
                                            }) |
                                    ranges_ext::min<float>();

        return {rv, num_traits};  // return map {species name, species object}
    }

    void TraitsProblemInputs::loadRobots(
            const std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping,
            const unsigned int num_traits,
            const nlohmann::json& j)
    {
        unsigned int num_robots = j.size();
        unsigned int robot_nr   = 0;

        m_robots.reserve(num_robots);
        m_team_traits_matrix.resize(num_robots, num_traits);
        for(const nlohmann::json robot_j: j)
        {
            const std::string name = robot_j.at(constants::k_name).get<std::string>();
            std::shared_ptr<const ConfigurationBase> initial_configuration =
                    robot_j.at(constants::k_initial_configuration).get<std::shared_ptr<ConfigurationBase>>();
            const std::string species_name = robot_j.at(constants::k_species).get<std::string>();

            const Eigen::VectorXf initial_trait_levels = robot_j.at(constants::k_initial_trait_levels).get<Eigen::VectorXf>();
            float initial_battery_level = robot_j.at(constants::k_initial_battery_level).get<float>();

            m_robots.push_back(
                    std::make_shared<const Robot>(name,
                                                      initial_configuration,
                                                      name_to_species_mapping.at(species_name),
                                                      initial_trait_levels,
                                                      initial_battery_level,
                                                      num_traits,
                                                      numberOfTasks()
                                                      ));

            m_team_traits_matrix.row(robot_nr++) = m_robots.back()->species()->traits_max();
        }
    }

    TimePoint TraitsProblemInputs::loadTimePoint(const nlohmann::json& timepoint_j)
    {
        TimePointType timepoint_type = TimePointType::e_null;
        if (timepoint_j.at(constants::k_timepoint_type) == constants::k_start) {
            timepoint_type = TimePointType::e_start;
        } else if (timepoint_j.at(constants::k_timepoint_type) == constants::k_completion) {
            timepoint_type = TimePointType::e_completion;
        } else {
            throw createLogicError("Incompatible TimePoint Type!");
        }
        int task_id = timepoint_j.at(constants::k_task).get<int>();
        return TimePoint(timepoint_type, task_id);
    }

    void TraitsProblemInputs::loadDeadlines(const nlohmann::json& j)
    {
        if (j.contains(constants::k_deadline_constraints))
        {
            m_deadlines.reserve(j.at(constants::k_deadline_constraints).size());
            for(const nlohmann::json& deadline_j: j.at(constants::k_deadline_constraints))
            {
                const std::string deadline_type = deadline_j.at(constants::k_deadline_type).get<std::string>();
                const float bound = deadline_j.at(constants::k_bound).get<float>();

                if (deadline_type == constants::k_absolute_deadline)
                {
                    if (!deadline_j.contains(constants::k_timepoint))
                    {
                        Logger::error("Must have timepoint");
                        break;
                    }
                    TimePoint timepoint = loadTimePoint(deadline_j.at(constants::k_timepoint));
                    m_deadlines.push_back(std::make_shared<AbsoluteDeadline>(timepoint, bound));
                }
                else if (deadline_type == constants::k_relative_deadline)
                {
                    if (!deadline_j.contains(constants::k_predecessor) || !deadline_j.contains(constants::k_successor))
                    {
                        Logger::error("Must have predecessor and successor timepoints");
                        break;
                    }
                    TimePoint predecessor = loadTimePoint(deadline_j.at(constants::k_predecessor));
                    TimePoint successor = loadTimePoint(deadline_j.at(constants::k_successor));
                    m_deadlines.push_back(std::make_shared<RelativeDeadline>(predecessor, successor, bound));
                }
                else
                {
                    Logger::error("Dealine must be either absolute or relative!");
                    break;
                }
            }
        }
    }

    TraitsPlanView TraitsProblemInputs::planTasks() const
    {
        return m_plan_task_indices | ::ranges::views::transform(std::function(
                [this](unsigned int i) -> const std::shared_ptr<const Task>&
                {
                    return task(i);
                }));
    }

    const std::shared_ptr<const Task>& TraitsProblemInputs::planTask(unsigned int index) const
    {
        return planTasks()[index];
    }

    std::vector<std::shared_ptr<const Task>> TraitsProblemInputs::loadTasks(const nlohmann::json& j,
                                std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping)
    {
        if(!j.is_array())
        {
            throw createLogicError("''tasks' must be an array");
        }

        std::vector<std::shared_ptr<const Task>> tasks;
        tasks.reserve(j.size());

        for(const nlohmann::json& task_j: j)
        {
            std::string name = "";
            if(const auto& name_j_itr = task_j.find(constants::k_name); name_j_itr != task_j.end())
            {
                name = *name_j_itr;
            }
            const float static_duration                    = task_j.at(constants::k_static_duration);
            const Eigen::VectorXf desired_traits           = task_j.at(constants::k_desired_traits);
            const Eigen::VectorXf minimum_trait_rates      = task_j.at(constants::k_minimum_trait_rates);
            const Eigen::VectorXf traits_aggregatable          = task_j.at(constants::k_traits_aggregatable);
            const nlohmann::json& initial_configuration_j  = task_j.at(constants::k_initial_configuration);
            const nlohmann::json& terminal_configuration_j = task_j.at(constants::k_terminal_configuration);

            const std::shared_ptr<const ConfigurationBase> initial_configuration =
                    initial_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(initial_configuration);

            const std::shared_ptr<const ConfigurationBase> terminal_configuration =
                    terminal_configuration_j.get<std::shared_ptr<ConfigurationBase>>();
            checkConfiguration(terminal_configuration);

            float maximum_dynamic_task_duration = 0.0f;
            for (unsigned int trait_nr = 0; trait_nr < desired_traits.rows(); ++trait_nr)
            {
                // if task trait is static
                if (minimum_trait_rates(trait_nr) < 1e-6) {
                    continue; // no effects on dynamic task time
                }

                // BELOW IS FOR DYNAMIC TRAITS
                float trait_duration = 0.0f;
                // If task requires all robots to meet the minimum trait rate requirements
                if (traits_aggregatable(trait_nr) < 0.5f) {
                    trait_duration = desired_traits(trait_nr) / minimum_trait_rates(trait_nr);
                }
                else {  // if coalition is allowed
                    // find the minimum trait rates from all species
                    for (const auto& kv: name_to_species_mapping)
                    {
                        const auto& species = kv.second;

                        if (species->traits_max()(trait_nr) < 1e-6) {  // if species doesn't support that trait
                            continue;  // has no effect on time
                        }

                        // if species support trait and rate is greater than zero
                        trait_duration = std::max(trait_duration,
                                                  desired_traits(trait_nr) / minimum_trait_rates(trait_nr));

                    }
                }
                maximum_dynamic_task_duration= std::max(maximum_dynamic_task_duration, trait_duration);
            }

            tasks.push_back(std::make_shared<const Task>(std::make_shared<SasAction>(name, static_duration),
                                                             desired_traits,
                                                             minimum_trait_rates,
                                                             traits_aggregatable,
                                                             initial_configuration,
                                                             terminal_configuration,
                                                             m_fastest_species_speed,
                                                             m_slowest_species_speed,
                                                             0.0f,
                                                             maximum_dynamic_task_duration));
        }
        return tasks;
    }

    bool TraitsProblemInputs::computeScheduleBestWorst()
    {
        // Compute makespan for schedule best
        // TODO(Andrew): turn this into a utility/static function somewhere
        {
            const float longest_path = motionPlanners() |
                                       ::ranges::views::transform(
                                               [](const std::shared_ptr<TraitsOmplMotionPlanner> &motion_planner) -> float {
                                                   return motion_planner->environment()->longestPath();
                                               }) |
                                       ranges_ext::max<float>();

            const float worst_mp_duration = longest_path / m_slowest_species_speed;
            m_schedule_worst_makespan     = planTasks() |
                                            ::ranges::views::transform(
                                                    [=](const std::shared_ptr<const Task>& task) -> float
                                                    {
                                                        return 2.0f * worst_mp_duration + task->staticDuration();
                                                    }) |
                                            ranges_ext::sum<float>();

            Logger::debug(fmt::format("worst schedule makespan: {}", m_schedule_worst_makespan));
        }
        {
            m_schedule_best_makespan = 0.0f;

            // Create empty allocation matrix
            Eigen::MatrixXf allocation(numberOfPlanTasks(), numberOfRobots());
            allocation.setZero();

            auto scheduler_problem_inputs = std::make_shared<TraitsSchedulerProblemInputs>(
                    std::shared_ptr<TraitsProblemInputs>(this, [](TraitsProblemInputs*) {}),
                    allocation,
                    SchedulerObjectiveType::e_heuristic_makespan_lb);

            TraitsMilpScheduler scheduler(scheduler_problem_inputs);
            std::shared_ptr<const TraitsSchedulerResult> scheduler_result = scheduler.solve();
            if(scheduler_result->failed())
            {
                throw createLogicError("Makespan LowerBound cannot be created. Problem is unsolvable.");
                // equivalent to return false;
            }

            m_schedule_best_makespan = scheduler_result->schedule()->makespan();
            Logger::debug(fmt::format("best schedule makespan: {}", m_schedule_best_makespan));
            scheduler.resetNumIterations();
        }
        MilpSolverBase::clearEnvironments();

        return true;  // feasible schedule exists -- LB and UB
    }

}  // namespace traits

namespace nlohmann
{
    std::shared_ptr<traits::TraitsProblemInputs>
    nlohmann::adl_serializer<std::shared_ptr<traits::TraitsProblemInputs>>::from_json(const nlohmann::json& j)
    {
        auto problem_inputs =
                std::make_shared<traits::TraitsProblemInputs>(traits::TraitsProblemInputs::s_this_is_protected_tag);

        std::vector<std::shared_ptr<traits::SasAction>> grounded_sas_actions;

        // Load Environments & Motion Planners
        problem_inputs->loadMotionPlanners(j.at(traits::constants::k_motion_planners));

        // Load Species (must load prior to tasks, because need to calculate the fastest species' speed)
        auto [name_to_species_mapping, num_traits] = problem_inputs->loadSpecies(j.at(traits::constants::k_species));

        // Create tasks (must load species first -- need the fastest species' speed)
        problem_inputs->m_tasks = problem_inputs->loadTasks(j.at(traits::constants::k_tasks), name_to_species_mapping);

        // Load Robots
        // Note: cannot use the normal from_json function because the vector of species is needed
        problem_inputs->loadRobots(name_to_species_mapping, num_traits, j.at(traits::constants::k_robots));

        // Load Deadlines
        problem_inputs->loadDeadlines(j);

        // Load Coefficients / Weights
        problem_inputs->m_alpha = j.at(traits::constants::k_traits_parameters).contains(traits::constants::k_alpha) ?
                j.at(traits::constants::k_traits_parameters).at(traits::constants::k_alpha).get<float>() : 0.5;
        problem_inputs->m_gamma= j.at(traits::constants::k_traits_parameters).contains(traits::constants::k_gamma) ?
                                 j.at(traits::constants::k_traits_parameters).at(traits::constants::k_gamma).get<float>() : 0.5;

        problem_inputs->m_timer_name = j.at(traits::constants::k_traits_parameters).contains(traits::constants::k_timer_name) ?
                j.at(traits::constants::k_traits_parameters).at(traits::constants::k_timer_name) : "traits";

        // Load Module Parameters
        {
            problem_inputs->m_traits_parameters =
                    traits::ParametersFactory::instance().create(traits::ParametersFactory::Type::e_search,
                                                                j.at(traits::constants::k_traits_parameters));
            if(j.find(traits::constants::k_robot_traits_matrix_reduction) != j.end())
            {
                problem_inputs->m_robot_traits_matrix_reduction =
                        j.at(traits::constants::k_robot_traits_matrix_reduction)
                                .get<std::shared_ptr<traits::RobotTraitsMatrixReduction>>();
            }
            else
            {
                problem_inputs->m_robot_traits_matrix_reduction =
                        std::make_shared<const traits::RobotTraitsMatrixReduction>();
            }
            problem_inputs->m_scheduler_parameters =
                    traits::ParametersFactory::instance().create(traits::ParametersFactory::Type::e_scheduler,
                                                                j.at(traits::constants::k_scheduler_parameters));
            problem_inputs->m_trait_distribution_parameters =
                    traits::ParametersFactory::instance().create(traits::ParametersFactory::Type::e_trait_distribution,
                                                               j.at(traits::constants::k_trait_distribution_parameters));
            // MP parameters are loaded up above
        }

        // Load Task Indices
        if(j.contains(traits::constants::k_plan_task_indices))
        {
            j.at(traits::constants::k_plan_task_indices).get_to(problem_inputs->m_plan_task_indices);
        }
        else
        {
            problem_inputs->m_plan_task_indices =
                    std::views::iota(0u, problem_inputs->numberOfTasks()) |
                    ::ranges::to<std::vector<unsigned int>>();
        }


        // set Timer
        traits::TimerRunner timer_runner(problem_inputs->m_timer_name);

        // Compute LB and UB Schedules
        std::set<std::pair<unsigned int, unsigned int>> tmp;
        j.at(traits::constants::k_precedence_constraints).get_to(tmp);
        problem_inputs->m_precedence_constraints = traits::addPrecedenceTransitiveConstraints(std::move(tmp));
        problem_inputs->m_desired_traits_matrix =
                desiredTraitsMatrix(problem_inputs->m_tasks, problem_inputs->m_plan_task_indices);
        problem_inputs->m_desired_trait_rates_matrix =
                desiredTraitRatesMatrix(problem_inputs->m_tasks, problem_inputs->m_plan_task_indices);
        problem_inputs->m_max_team_traits_matrix = maxTeamTraitsMatrix(problem_inputs->m_robots);
        problem_inputs->m_max_team_trait_rates_matrix = maxTeamTraitRatesMatrix(problem_inputs->m_robots);

        if (problem_inputs->computeScheduleBestWorst() == false) {
            return nullptr;
        }

        return problem_inputs;
    }
}  // namespace nlohmann
