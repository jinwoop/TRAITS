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
#include <vector>
#include <set>
// External
#include <Eigen/Core>
#include <nlohmann/json.hpp>
// Local
#include "traits/common/utilities/custom_views.hpp"
#include "problem_inputs.hpp"
#include "traits/common/utilities/timer_runner.hpp"

namespace traits
{
    // Modified Forward Declarations
    class TraitsSchedulerProblemInputs;
    class Robot;
    class Species;
    class Task;

    // Unmodified Forward Declarations
    class ConfigurationBase;
    class EnvironmentBase;
    class TraitsOmplMotionPlanner;
    class RobotTraitsMatrixReduction;
    class SasAction;
    class ParametersBase;

    class DeadlineBase;
    class TimePoint;

    enum class ConfigurationType : uint8_t;
    enum class OmplStateSpaceType : uint8_t;
    enum class GraphType : uint8_t;
    enum class SchedulerObjectiveType : uint8_t;

    //! A container for input for a Priority based Task Allocation and Scheduling problem
    class TraitsProblemInputs : public ProblemInputs
    {
    public:
        /*!
         * \brief Acts as a protected default constructor. Used for json deserialization
         */
        explicit TraitsProblemInputs(const ThisIsProtectedTag&);

        //! Destructor
        ~TraitsProblemInputs() override;

        // From ITAGS
        [[nodiscard]] TraitsPlanView planTasks() const;
        [[nodiscard]] const std::shared_ptr<const Task>& planTask(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfPlanTasks() const;
        [[nodiscard]] inline const std::set<std::pair<unsigned int, unsigned int>>& precedenceConstraints() const;
        [[nodiscard]] inline const Eigen::MatrixXf& desiredTraitsMatrix() const;
        [[nodiscard]] inline const Eigen::MatrixXf& desiredTraitRatesMatrix() const;
        [[nodiscard]] inline const Eigen::MatrixXf& maxTeamTraitsMatrix() const;
        [[nodiscard]] inline const Eigen::MatrixXf& maxTeamTraitRatesMatrix() const;
        [[nodiscard]] inline float scheduleBestMakespan() const;
        [[nodiscard]] inline float scheduleWorstMakespan() const;

        // TRAITS only
        [[nodiscard]] inline float alpha() const;
        [[nodiscard]] inline float gamma() const;
                      inline void setAlpha(float alpha);
        [[nodiscard]] inline float fastestSpeed() const;
        [[nodiscard]] inline float slowestSpeed() const;
        [[nodiscard]] inline const std::vector<std::shared_ptr<const DeadlineBase>>& deadlines() const;

        // Module Parameters
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& TraitsParameters() const;
        [[nodiscard]] inline const std::shared_ptr<const RobotTraitsMatrixReduction>& robotTraitsMatrixReduction()
        const;
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& schedulerParameters() const;
        [[nodiscard]] inline const std::shared_ptr<const ParametersBase>& traitDistributionParameters() const;

        // Tasks
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Task>>& tasks() const;
        [[nodiscard]] inline const std::shared_ptr<const Task>& task(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfTasks() const;
        // Robots
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Robot>>& robots() const;
        [[nodiscard]] inline const std::shared_ptr<const Robot>& robot(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfRobots() const;
        // Species
        [[nodiscard]] inline const std::vector<std::shared_ptr<const Species>>& multipleSpecies() const;
        [[nodiscard]] inline const std::shared_ptr<const Species>& individualSpecies(unsigned int index) const;
        [[nodiscard]] inline unsigned int numberOfSpecies() const;
        // Team Traits Matrix
        [[nodiscard]] inline const Eigen::MatrixXf& teamTraitsMatrix() const;
        [[nodiscard]] inline unsigned int numberOfTraits() const;
        // Motion Planners
        [[nodiscard]] inline const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& motionPlanners() const;
        [[nodiscard]] inline const std::shared_ptr<TraitsOmplMotionPlanner>& motionPlanner(unsigned int index) const;

        [[nodiscard]] inline const std::string& timerName() const;

        //! Checks if \p configuration matches the previously loaded environments
        void checkConfiguration(const std::shared_ptr<const ConfigurationBase>& configuration) const;

    protected:
        std::vector<std::shared_ptr<const Task>> loadTasks(const nlohmann::json& j,
                                   std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping);

        //! compute the values for m_schedule_best_makespan and m_schedule_worst_makespan
        bool computeScheduleBestWorst();

        //! Load motion planners from json
        void loadMotionPlanners(const nlohmann::json& j);

        //! Merges symbolic actions with non-symbolic information
        void createTasks(const std::vector<std::shared_ptr<SasAction>>& grounded_sas_actions, const nlohmann::json& j);

        //! Loads species from json
        std::pair<std::map<std::string, std::shared_ptr<const Species>>, unsigned int> loadSpecies(
                const nlohmann::json& j);

        //! Loads robots from json
        void loadRobots(const std::map<std::string, std::shared_ptr<const Species>>& name_to_species_mapping,
                        const unsigned int num_traits,
                        const nlohmann::json& j);

        //! Load Deadline Constraints
        void loadDeadlines(const nlohmann::json& j);

        //! Load Time Points for Deadlines
        TimePoint loadTimePoint(const nlohmann::json& timepoint_j);

        // From task planning
        std::vector<unsigned int> m_plan_task_indices;
        std::set<std::pair<unsigned int, unsigned int>> m_precedence_constraints;
        Eigen::MatrixXf m_desired_traits_matrix;
        Eigen::MatrixXf m_desired_trait_rates_matrix;

        Eigen::MatrixXf m_max_team_traits_matrix;
        Eigen::MatrixXf m_max_team_trait_rates_matrix;

        float m_schedule_best_makespan;
        float m_schedule_worst_makespan;
        float m_fastest_species_speed;
        float m_slowest_species_speed;
        float m_alpha;
        float m_gamma;

        // Module Parameters
        std::shared_ptr<const ParametersBase> m_traits_parameters;
        std::shared_ptr<const RobotTraitsMatrixReduction> m_robot_traits_matrix_reduction;
        std::shared_ptr<const ParametersBase> m_scheduler_parameters;
        std::shared_ptr<const ParametersBase> m_trait_distribution_parameters;
        // MP parameters are passed directly to the individual motion planners

        // Problem Inputs
        std::vector<std::shared_ptr<const Task>> m_tasks;
        std::vector<std::shared_ptr<const Robot>> m_robots;
        std::vector<std::shared_ptr<const Species>> m_species;
        std::vector<std::shared_ptr<const DeadlineBase>> m_deadlines;
        Eigen::MatrixXf m_team_traits_matrix;
        std::vector<std::shared_ptr<TraitsOmplMotionPlanner>> m_motion_planners;

        ConfigurationType m_task_configuration_type;
        OmplStateSpaceType m_ompl_state_space_type;
        GraphType m_graph_type;
        std::string m_timer_name;

        // required to have itself friend
        friend struct nlohmann::adl_serializer<std::shared_ptr<TraitsProblemInputs>>;
        friend struct nlohmann::adl_serializer<std::shared_ptr<TraitsSchedulerProblemInputs>>;
    };

    // Inlined Functions
    const std::string& TraitsProblemInputs::timerName() const
    {
       return m_timer_name;
    }

    const std::vector<std::shared_ptr<const DeadlineBase>>& TraitsProblemInputs::deadlines() const
    {
       return m_deadlines;
    }

    float TraitsProblemInputs::fastestSpeed() const
    {
        return m_fastest_species_speed;
    }

    float TraitsProblemInputs::slowestSpeed() const
    {
        return m_slowest_species_speed;
    }

    float TraitsProblemInputs::alpha() const {
        return m_alpha;
    }

    float TraitsProblemInputs::gamma() const {
        return m_gamma;
    }

    void TraitsProblemInputs::setAlpha(float alpha) {
        m_alpha = alpha;
    }

    unsigned int TraitsProblemInputs::numberOfPlanTasks() const
    {
        return m_plan_task_indices.size();
    }

    const std::set<std::pair<unsigned int, unsigned int>>& TraitsProblemInputs::precedenceConstraints() const
    {
        return m_precedence_constraints;
    }

    const Eigen::MatrixXf& TraitsProblemInputs::desiredTraitsMatrix() const
    {
        return m_desired_traits_matrix;
    }

    const Eigen::MatrixXf& TraitsProblemInputs::desiredTraitRatesMatrix() const
    {
        return m_desired_trait_rates_matrix;
    }

    const Eigen::MatrixXf& TraitsProblemInputs::maxTeamTraitsMatrix() const
    {
        return m_max_team_traits_matrix;
    }

    const Eigen::MatrixXf& TraitsProblemInputs::maxTeamTraitRatesMatrix() const
    {
        return m_max_team_trait_rates_matrix;
    }

    float TraitsProblemInputs::scheduleBestMakespan() const
    {
        return m_schedule_best_makespan;
    }

    float TraitsProblemInputs::scheduleWorstMakespan() const
    {
        return m_schedule_worst_makespan;
    }

    const std::shared_ptr<const ParametersBase>& TraitsProblemInputs::TraitsParameters() const
    {
        return m_traits_parameters;
    }

    const std::shared_ptr<const RobotTraitsMatrixReduction>& TraitsProblemInputs::robotTraitsMatrixReduction() const
    {
        return m_robot_traits_matrix_reduction;
    }

    const std::shared_ptr<const ParametersBase>& TraitsProblemInputs::schedulerParameters() const
    {
        return m_scheduler_parameters;
    }

    const std::shared_ptr<const ParametersBase>& TraitsProblemInputs::traitDistributionParameters() const
    {
        return m_trait_distribution_parameters;
    }

    const std::vector<std::shared_ptr<const Task>>& TraitsProblemInputs::tasks() const
    {
        return m_tasks;
    }

    const std::shared_ptr<const Task>& TraitsProblemInputs::task(unsigned int index) const
    {
        assert(index < m_tasks.size());
        return m_tasks[index];
    }

    unsigned int TraitsProblemInputs::numberOfTasks() const
    {
        return m_tasks.size();
    }

    const std::vector<std::shared_ptr<const Robot>>& TraitsProblemInputs::robots() const
    {
        return m_robots;
    }

    const std::shared_ptr<const Robot>& TraitsProblemInputs::robot(unsigned int index) const
    {
        assert(index < m_robots.size());
        return m_robots[index];
    }

    unsigned int TraitsProblemInputs::numberOfRobots() const
    {
        return m_robots.size();
    }

    const std::vector<std::shared_ptr<const Species>>& TraitsProblemInputs::multipleSpecies() const
    {
        return m_species;
    }

    const std::shared_ptr<const Species>& TraitsProblemInputs::individualSpecies(unsigned int index) const
    {
        assert(index < m_species.size());
        return m_species[index];
    }

    unsigned int TraitsProblemInputs::numberOfSpecies() const
    {
        return m_species.size();
    }

    const Eigen::MatrixXf& TraitsProblemInputs::teamTraitsMatrix() const
    {
        return m_team_traits_matrix;
    }

    unsigned int TraitsProblemInputs::numberOfTraits() const
    {
        return m_team_traits_matrix.cols();
    }

    const std::vector<std::shared_ptr<TraitsOmplMotionPlanner>>& TraitsProblemInputs::motionPlanners() const
    {
        return m_motion_planners;
    }

    const std::shared_ptr<TraitsOmplMotionPlanner>& TraitsProblemInputs::motionPlanner(unsigned int index) const
    {
        assert(index < m_motion_planners.size());
        return m_motion_planners[index];
    }
}  // namespace traits

namespace nlohmann
{
    template <>
    struct adl_serializer<std::shared_ptr<traits::TraitsProblemInputs>>
    {
        static std::shared_ptr<traits::TraitsProblemInputs> from_json(const nlohmann::json& j);
    };
}  // namespace nlohmann